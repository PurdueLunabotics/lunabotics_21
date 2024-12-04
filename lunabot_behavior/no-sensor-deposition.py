#!/usr/bin/env python3

import rospy

from lunabot_msgs.msg import RobotSensors
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

from homing_controller import HomingController

import time
import interrupts
import sys

class Deposition:
    '''
    State to deposit collected regolith onto the berm by spinning the auger
    '''

    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, deposition_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if deposition_publisher is None:
            self.deposition_publisher = rospy.Publisher("/deposition", Int32, queue_size=1, latch=True)
            rospy.init_node('deposition_node')
        else:
            self.deposition_publisher = deposition_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)  # 10hz

        self.deposition_RPM = 1000 # RPM

        # self.load_cell_threshold = 1 # in kilograms, TODO test / verify WHEN LOAD CELLS EXIST

        self.DEPOSITION_TIME = 45.00

        self.is_sim = rospy.get_param("is_sim")

    def deposit(self):
        """
        Spin the auger until the load cell detects that the bin is empty
        """

        if (self.is_sim):
            rospy.loginfo("Deposition: would deposit")
            time.sleep(2)
            return True

        time.sleep(0.1)

        deposition_msg = Int32()
        deposition_msg.data = self.deposition_RPM

        start_time = rospy.get_time()

        while True:
            self.deposition_publisher.publish(deposition_msg)
            
            if rospy.get_time() - start_time > self.DEPOSITION_TIME:
                break

            '''
            # no load cells yet... :(

            weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

            if weight < self.load_cell_threshold:
                break
            '''

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False

            self.rate.sleep()

        deposition_msg.data = 0
        self.deposition_publisher.publish(deposition_msg)

        return True
    
if __name__ == "__main__":

    start_time = rospy.get_time()

    DRIVE_TIME = 5.0

    cmd_vel = Twist()
    cmd_vel.linear.x = -0.3
    cmd_vel.angular.z = 0

    cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)

    time.sleep(0.1)

    while rospy.get_time() - start_time < DRIVE_TIME:
        cmd_vel_publisher.publish(cmd_vel)

    cmd_vel.linear.x = 0
    cmd_vel_publisher.publish(cmd_vel)

    time.sleep(0.1)

    deposition = Deposition()

    deposition.deposit()
