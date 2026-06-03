#ifndef DOG_ROBOT_BRIDGE_PROTOCOL_H_
#define DOG_ROBOT_BRIDGE_PROTOCOL_H_

#include <stdint.h>

#pragma pack(push, 4)
namespace QNX2ROSProtocol
{

struct ImuData
{
  uint32_t timestamp;
  union
  {
    float buffer_float[9];
    uint8_t buffer_byte[3][12];
    struct
    {
      float angle_roll, angle_pitch, angle_yaw;
      float angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw;
      float acc_x, acc_y, acc_z;
    };
  };
};

struct ImuDataReceived
{
  int code;
  int size;
  int cons_code;
  struct ImuData data;
};

struct RobotState
{
  int robot_basic_state;
  int robot_gait_state;
  double rpy[3];
  double rpy_vel[3];
  double xyz_acc[3];
  double pos_world[3];
  double vel_world[3];
  double vel_body[3];
  unsigned touch_down_and_stair_trot;
  bool is_charging;
  unsigned error_state;
  int robot_motion_state;
  double battery_level;
  int task_state;
  bool is_robot_need_move;
  bool zero_position_flag;
  double ultrasound[2];
};

struct RobotStateReceived
{
  int code;
  int size;
  int cons_code;
  struct RobotState data;
};

struct JointState
{
  double LF_Joint;
  double LF_Joint_1;
  double LF_Joint_2;
  double RF_Joint;
  double RF_Joint_1;
  double RF_Joint_2;
  double LB_Joint;
  double LB_Joint_1;
  double LB_Joint_2;
  double RB_Joint;
  double RB_Joint_1;
  double RB_Joint_2;
};

struct JointStateReceived
{
  int code;
  int size;
  int cons_code;
  struct JointState data;
};

struct HandleState
{
  double left_axis_forward;
  double left_axis_side;
  double right_axis_yaw;
  double goal_vel_forward;
  double goal_vel_side;
  double goal_vel_yaw;
};

struct HandleStateReceived
{
  int code;
  int size;
  int cons_code;
  struct HandleState data;
};

}
#pragma pack(pop)

#endif
