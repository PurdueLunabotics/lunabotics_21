import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from lunabot_msgs.msg import RobotEffort, RobotErrors, RobotSensors

import time

'''
A node that detects if the robot is stuck (trying to move, and not moving), and publishes to the error topic.
TODO add this to roslaunch so it always runs
'''
class Stuck:
    MIN_STUCK_TIME = 3.0 # seconds

    def robot_sensors_callback(self, msg):
        self.robot_sensors = msg


    def effort_callback(self, msg):
        self.robot_effort = msg


    def __init__(self):
        self.robot_sensors = RobotSensors()
        self.robot_effort = RobotEffort()

    def stuck(self):
        time.sleep(0.1)

        rospy.init_node("stuck_node")

        error_publisher = rospy.Publisher("/errors", RobotErrors, queue_size=1)
        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/sensors", RobotSensors, self.robot_sensors_callback)

        stuck_msg = RobotErrors()
        stuck_msg.stuck = False
        error_publisher.publish(stuck_msg)

        currently_stuck = False
        stuck_time = rospy.get_time()

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if rospy.get_time() - stuck_time >= self.MIN_STUCK_TIME and currently_stuck:
                stuck_msg.stuck = True
            else:
                stuck_msg.stuck = False

            error_publisher.publish(stuck_msg)

            # check if trying to go somewhere
            if self.robot_effort.left_drive >= 15 or self.robot_effort.right_drive >= 300:
                # check if not going anywhere
                if self.robot_sensors.drive_left_vel <= 0.1 and self.robot_sensors.drive_right_vel <= 0.1:
                    if not currently_stuck:
                        currently_stuck = True
                        stuck_time = rospy.get_time()
                    continue

            currently_stuck = False
            rate.sleep()

        stuck_msg.stuck = False
        error_publisher.publish(stuck_msg)

if __name__ == "__main__":
    stuck = Stuck()
    stuck.stuck()
    rospy.spin()