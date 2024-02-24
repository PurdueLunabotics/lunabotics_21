import rospy

import ascent
import escape
import find_apriltag
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors

class Behavior:

    def robot_state_callback(self, msg: RobotSensors):
        self.robot_state = msg

    def effort_callback(self, msg: RobotEffort):
        self.robot_effort = msg

    def errors_callback(self, msg: RobotErrors):
        self.robot_errors = msg

    def __init__(self):
        self.robot_state: RobotSensors = RobotSensors()
        self.robot_effort: RobotEffort = RobotEffort()
        self.robot_errors: RobotErrors = RobotErrors()

        self.found_apriltag = False

        self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)

        # TODO change to parameters
        rospy.Subscriber("/sensors", RobotSensors, self.robot_state_callback)
        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/errors", RobotErrors, self.errors_callback)

        rospy.init_node('behavior_node')

    def behavior_loop():

        #create interrupt logic here!
        # rospy.is_shutdown():
        # stuck
        # map change
        # overcurrent

        # notes on interrupts: if no functions use rospy.sleep, then they can all 
        # just check for the kill signal at the end of every loop they go through. 
        # that can cause them to return false or the error in an enum (need to decide)
        # and that can be handled by the main process before continuing. 
        # How to pick up where we were before? Current state enum flag?
        # also how to cleanly handle the functions returning different values?
        # and picking back up from somewhere new specificed by the enum flag?

        #startup stuff here
        ascent.main()
        find_apriltag.main()

        while(True):
            #traverse to mining zone

            #plunge

            #trench

            ascent.main()

            #berm plan

            #go to berm

            #deposit

            #post_deposit

            ascent.main() #why?

            #plan to mining zone

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()