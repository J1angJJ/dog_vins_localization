#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <string>

#include <boost/array.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include "dog_robot_bridge/protocol.h"

namespace
{
const double kPi = 3.14159265358979323846;

double degToRad(double deg)
{
  return deg * kPi / 180.0;
}

void fillUnknownCovariance(boost::array<double, 36>& covariance)
{
  for (size_t i = 0; i < covariance.size(); ++i)
    covariance[i] = 0.0;
  covariance[0] = 0.05 * 0.05;
  covariance[7] = 0.05 * 0.05;
  covariance[14] = 0.10 * 0.10;
  covariance[21] = 0.20 * 0.20;
  covariance[28] = 0.20 * 0.20;
  covariance[35] = 0.10 * 0.10;
}
}

class DogQnxStateBridge
{
public:
  explicit DogQnxStateBridge(const ros::NodeHandle& nh)
      : nh_(nh)
  {
    ros::NodeHandle pnh("~");
    pnh.param("server_port", server_port_, 43897);
    pnh.param("publish_imu", publish_imu_, true);
    pnh.param("publish_joint_states", publish_joint_states_, false);
    pnh.param("publish_aux_states", publish_aux_states_, true);
    pnh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    pnh.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    pnh.param<std::string>("imu_frame_id", imu_frame_id_, "base_link");

    openSocket();

    leg_odom_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/leg_odom", 5);
    leg_odom2_pub_ = nh_.advertise<nav_msgs::Odometry>("/leg_odom2", 5);
    if (publish_imu_)
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 20);
    if (publish_joint_states_)
      joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 5);
    if (publish_aux_states_)
    {
      ultrasound_pub_ = nh_.advertise<std_msgs::Float64>("/us_publisher/ultrasound_distance", 5);
      robot_basic_state_pub_ = nh_.advertise<std_msgs::Int32>("/lite3/robot_basic_state", 5);
      robot_gait_state_pub_ = nh_.advertise<std_msgs::Int32>("/lite3/robot_gait_state", 5);
      robot_motion_state_pub_ = nh_.advertise<std_msgs::Int32>("/lite3/robot_motion_state", 5);
      robot_battery_level_pub_ = nh_.advertise<std_msgs::Float64>("/lite3/battery_level", 5);
      robot_need_move_pub_ = nh_.advertise<std_msgs::Bool>("/lite3/is_robot_need_move", 5);
    }

    ROS_INFO("dog_qnx_state_bridge listening on UDP port %d", server_port_);
  }

  ~DogQnxStateBridge()
  {
    if (socket_fd_ >= 0)
      close(socket_fd_);
  }

  void spin()
  {
    ros::Rate idle_rate(10000);
    while (ros::ok())
    {
      receiveOnce();
      ros::spinOnce();
      idle_rate.sleep();
    }
  }

private:
  void openSocket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
      ROS_FATAL("socket failed: %s", strerror(errno));
      ros::shutdown();
      return;
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&listen_addr_, 0, sizeof(listen_addr_));
    listen_addr_.sin_family = AF_INET;
    listen_addr_.sin_port = htons(server_port_);
    listen_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&listen_addr_), sizeof(listen_addr_)) < 0)
    {
      ROS_FATAL("bind UDP port %d failed: %s", server_port_, strerror(errno));
      ros::shutdown();
    }
  }

  void receiveOnce()
  {
    if (socket_fd_ < 0)
    {
      ros::Duration(0.1).sleep();
      return;
    }

    socklen_t len = sizeof(listen_addr_);
    const ssize_t recv_num = recvfrom(socket_fd_, receive_buffer_, sizeof(receive_buffer_), 0,
                                      reinterpret_cast<struct sockaddr*>(&listen_addr_), &len);
    if (recv_num < 0)
    {
      if (errno != EINTR)
        ROS_WARN_THROTTLE(2.0, "recvfrom failed: %s", strerror(errno));
      return;
    }

    parseFrame(static_cast<size_t>(recv_num));
    packet_count_++;
    ROS_INFO_THROTTLE(5.0, "dog_qnx_state_bridge received packets: %llu",
                      static_cast<unsigned long long>(packet_count_));
  }

  void parseFrame(size_t recv_num)
  {
    if (recv_num == sizeof(QNX2ROSProtocol::RobotStateReceived))
      parseRobotStateFrame();
    else if (recv_num == sizeof(QNX2ROSProtocol::ImuDataReceived))
      parseImuDataFrame();
    else if (publish_joint_states_ && recv_num == sizeof(QNX2ROSProtocol::JointStateReceived))
      parseJointStateFrame();
    else
      ROS_DEBUG("ignored UDP packet length: %zu", recv_num);
  }

  void parseRobotStateFrame()
  {
    const QNX2ROSProtocol::RobotStateReceived* frame =
        reinterpret_cast<const QNX2ROSProtocol::RobotStateReceived*>(receive_buffer_);
    if (frame->code != 2305)
      return;

    const QNX2ROSProtocol::RobotState& state = frame->data;
    const ros::Time stamp = ros::Time::now();
    const geometry_msgs::Quaternion yaw_q = tf::createQuaternionMsgFromYaw(degToRad(state.rpy[2]));

    geometry_msgs::PoseWithCovarianceStamped leg_odom;
    leg_odom.header.stamp = stamp;
    leg_odom.header.frame_id = odom_frame_id_;
    leg_odom.pose.pose.position.x = state.pos_world[0];
    leg_odom.pose.pose.position.y = state.pos_world[1];
    leg_odom.pose.pose.position.z = state.pos_world[2];
    leg_odom.pose.pose.orientation = yaw_q;
    fillUnknownCovariance(leg_odom.pose.covariance);
    leg_odom_pub_.publish(leg_odom);

    nav_msgs::Odometry odom;
    odom.header = leg_odom.header;
    odom.child_frame_id = base_frame_id_;
    odom.pose = leg_odom.pose;
    odom.twist.twist.linear.x = state.vel_body[0];
    odom.twist.twist.linear.y = state.vel_body[1];
    odom.twist.twist.linear.z = state.vel_body[2];
    odom.twist.twist.angular.x = state.rpy_vel[0];
    odom.twist.twist.angular.y = state.rpy_vel[1];
    odom.twist.twist.angular.z = state.rpy_vel[2];
    fillUnknownCovariance(odom.twist.covariance);
    leg_odom2_pub_.publish(odom);

    if (publish_aux_states_)
      publishAuxStates(state);
  }

  void parseImuDataFrame()
  {
    if (!publish_imu_)
      return;

    const QNX2ROSProtocol::ImuDataReceived* frame =
        reinterpret_cast<const QNX2ROSProtocol::ImuDataReceived*>(receive_buffer_);
    if (frame->code != 0x010901)
      return;

    const QNX2ROSProtocol::ImuData& data = frame->data;
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = imu_frame_id_;
    const tf::Quaternion q = tf::createQuaternionFromRPY(degToRad(data.angle_roll),
                                                         degToRad(data.angle_pitch),
                                                         degToRad(data.angle_yaw));
    tf::quaternionTFToMsg(q, imu.orientation);
    imu.angular_velocity.x = data.angular_velocity_roll;
    imu.angular_velocity.y = data.angular_velocity_pitch;
    imu.angular_velocity.z = data.angular_velocity_yaw;
    imu.linear_acceleration.x = data.acc_x;
    imu.linear_acceleration.y = data.acc_y;
    imu.linear_acceleration.z = data.acc_z;
    imu_pub_.publish(imu);
  }

  void parseJointStateFrame()
  {
    const QNX2ROSProtocol::JointStateReceived* frame =
        reinterpret_cast<const QNX2ROSProtocol::JointStateReceived*>(receive_buffer_);
    if (frame->code != 2306)
      return;

    const QNX2ROSProtocol::JointState& state = frame->data;
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);

    const char* names[12] = {"LF_Joint", "LF_Joint_1", "LF_Joint_2",
                             "RF_Joint", "RF_Joint_1", "RF_Joint_2",
                             "LB_Joint", "LB_Joint_1", "LB_Joint_2",
                             "RB_Joint", "RB_Joint_1", "RB_Joint_2"};
    const double values[12] = {-state.LF_Joint, -state.LF_Joint_1, -state.LF_Joint_2,
                               -state.RF_Joint, -state.RF_Joint_1, -state.RF_Joint_2,
                               -state.LB_Joint, -state.LB_Joint_1, -state.LB_Joint_2,
                               -state.RB_Joint, -state.RB_Joint_1, -state.RB_Joint_2};
    for (size_t i = 0; i < 12; ++i)
    {
      joint_state.name[i] = names[i];
      joint_state.position[i] = values[i];
    }
    joint_state_pub_.publish(joint_state);
  }

  void publishAuxStates(const QNX2ROSProtocol::RobotState& state)
  {
    std_msgs::Float64 ultrasound;
    ultrasound.data = state.ultrasound[1];
    ultrasound_pub_.publish(ultrasound);

    std_msgs::Int32 basic_state;
    basic_state.data = state.robot_basic_state;
    robot_basic_state_pub_.publish(basic_state);

    std_msgs::Int32 gait_state;
    gait_state.data = state.robot_gait_state;
    robot_gait_state_pub_.publish(gait_state);

    std_msgs::Int32 motion_state;
    motion_state.data = state.robot_motion_state;
    robot_motion_state_pub_.publish(motion_state);

    std_msgs::Float64 battery_level;
    battery_level.data = state.battery_level;
    robot_battery_level_pub_.publish(battery_level);

    std_msgs::Bool need_move;
    need_move.data = state.is_robot_need_move;
    robot_need_move_pub_.publish(need_move);
  }

  ros::NodeHandle nh_;
  int server_port_;
  bool publish_imu_;
  bool publish_joint_states_;
  bool publish_aux_states_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string imu_frame_id_;

  int socket_fd_ = -1;
  struct sockaddr_in listen_addr_;
  uint8_t receive_buffer_[512];
  uint64_t packet_count_ = 0;

  ros::Publisher leg_odom_pub_;
  ros::Publisher leg_odom2_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher ultrasound_pub_;
  ros::Publisher robot_basic_state_pub_;
  ros::Publisher robot_gait_state_pub_;
  ros::Publisher robot_motion_state_pub_;
  ros::Publisher robot_battery_level_pub_;
  ros::Publisher robot_need_move_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dog_qnx_state_bridge");
  ros::NodeHandle nh;
  DogQnxStateBridge bridge(nh);
  bridge.spin();
  return 0;
}
