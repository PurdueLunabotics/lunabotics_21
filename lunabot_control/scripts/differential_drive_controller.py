#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from lunabot_msgs.msg import RobotSensors



class DifferentialDriveController:
    def __init__(self):
        # ROS Publishers and subsribers to get / send data

        self._vel_sub = rospy.Subscriber("/cmd_vel", Twist, self._vel_cb)
        self._right_drive_pub = rospy.Publisher("/right_drive", Int32, queue_size=1)
        self._left_drive_pub = rospy.Publisher("/left_drive", Int32, queue_size=1)
        self._state_sub = rospy.Subscriber("sensors", RobotSensors, self._robot_state_cb)

        self.width = rospy.get_param("~width", 0.5588)
        self.max_speed_percentage = rospy.get_param("~max_speed_percentage", 0.8)
        self.hz = rospy.get_param("~hz", 20)

        self._max_speed = rospy.get_param("~max_speed", 2.0) * 60 / 2 / np.pi # In rad/s converted to RPM
        
        self.lin = 0
        self.ang = 0

        self._left_vel = 0
        self._right_vel = 0
        self._meters_per_rad = 0.1397
        self._left_prev_error = 0
        self._right_prev_error = 0

        self.left_error_sum = 0
        self.right_error_sum = 0

        # Variables for PIDF Velocity Control

        self.loop_dt = 1 / self.hz  # Amount of time between calls of this function
        rate = rospy.Rate(self.hz)
        rospy.on_shutdown(self.shutdown_hook)

        while not rospy.is_shutdown():
            if (rospy.get_param("autonomy")):
                self._loop()
            rate.sleep()

    def _vel_cb(self, vel_msg):
        self.lin = vel_msg.linear.x
        self.ang = vel_msg.angular.z

    def _robot_state_cb(self, msg):
        self._right_vel = msg.drive_right_vel
        self._left_vel = msg.drive_left_vel

    def _loop(self):
        left_drive_msg = Int32()
        right_drive_msg = Int32()

        WEIGHT = 0.9

        self.lin *= WEIGHT
        self.ang *= 1/WEIGHT

        left_set = self.lin - self.ang * self.width / 2
        right_set = self.lin + self.ang * self.width / 2

        left_drive_msg.data = self.constrain(left_set * self._meters_per_rad)
        right_drive_msg.data = self.constrain(right_set * self._meters_per_rad)

        if self.lin == 0 and self.ang == 0:
            left_drive_msg.data = 0
            right_drive_msg.data = 0
        self._left_drive_pub.publish(left_drive_msg)
        self._right_drive_pub.publish(right_drive_msg)

    def constrain(self, val):
        val = np.clip(-self._max_speed, val, self._max_speed)  # Clipping speed to not go over 100%
        return np.int32(val)

    def shutdown_hook(self):
        left_drive_msg = Int32()
        right_drive_msg = Int32()

        left_drive_msg.data = 0
        right_drive_msg.data = 0

        self._left_drive_pub.publish(left_drive_msg)
        self._right_drive_pub.publish(right_drive_msg)
        rospy.loginfo("Differential Drive Controller: Stopping")


if __name__ == "__main__":
    rospy.init_node("differential_drive_controller")
    controller = DifferentialDriveController()
