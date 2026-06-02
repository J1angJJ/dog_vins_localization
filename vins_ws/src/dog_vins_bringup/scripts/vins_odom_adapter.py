#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _yaw_from_quat(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class VinsOdomAdapter:
    def __init__(self):
        self.output_frame = rospy.get_param("~output_frame", "odom")
        self.output_child_frame = rospy.get_param("~output_child_frame", "base_link")
        self.position_scale = float(rospy.get_param("~position_scale", 1.0))
        self.yaw_offset = float(rospy.get_param("~yaw_offset", 0.0))
        self.zero_on_first_pose = bool(rospy.get_param("~zero_on_first_pose", True))
        self.planar = bool(rospy.get_param("~planar", True))
        self.pose_covariance_diagonal = rospy.get_param(
            "~pose_covariance_diagonal",
            [0.05, 0.05, 1.0, 1.0, 1.0, 0.1],
        )
        self.twist_covariance_diagonal = rospy.get_param(
            "~twist_covariance_diagonal",
            [0.2, 0.2, 1.0, 1.0, 1.0, 0.5],
        )

        input_topic = rospy.get_param("~input_odom", "/dog_vins/vins_estimator/odometry")
        output_topic = rospy.get_param("~output_odom", "/dog_vins/odom_base_link")

        self.initial_x = None
        self.initial_y = None
        self.initial_z = None
        self.initial_yaw = None

        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=20)
        self.sub = rospy.Subscriber(input_topic, Odometry, self.callback, queue_size=50)
        rospy.loginfo("vins_odom_adapter: %s -> %s", input_topic, output_topic)

    def callback(self, msg):
        src_pos = msg.pose.pose.position
        src_ori = msg.pose.pose.orientation
        yaw = _yaw_from_quat(src_ori)

        if self.initial_x is None:
            self.initial_x = src_pos.x
            self.initial_y = src_pos.y
            self.initial_z = src_pos.z
            self.initial_yaw = yaw

        if self.zero_on_first_pose:
            x = src_pos.x - self.initial_x
            y = src_pos.y - self.initial_y
            z = src_pos.z - self.initial_z
            yaw = _wrap_angle(yaw - self.initial_yaw)
        else:
            x = src_pos.x
            y = src_pos.y
            z = src_pos.z

        x *= self.position_scale
        y *= self.position_scale
        z *= self.position_scale
        yaw = _wrap_angle(yaw + self.yaw_offset)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.output_child_frame
        out.pose = msg.pose
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = 0.0 if self.planar else z

        if self.planar:
            q = quaternion_from_euler(0.0, 0.0, yaw)
            out.pose.pose.orientation.x = q[0]
            out.pose.pose.orientation.y = q[1]
            out.pose.pose.orientation.z = q[2]
            out.pose.pose.orientation.w = q[3]

        out.twist = msg.twist
        self._set_diagonal(out.pose.covariance, self.pose_covariance_diagonal)
        self._set_diagonal(out.twist.covariance, self.twist_covariance_diagonal)
        self.pub.publish(out)

    @staticmethod
    def _set_diagonal(covariance, diagonal):
        for i, value in enumerate(diagonal[:6]):
            covariance[i * 6 + i] = float(value)


if __name__ == "__main__":
    rospy.init_node("vins_odom_adapter")
    VinsOdomAdapter()
    rospy.spin()
