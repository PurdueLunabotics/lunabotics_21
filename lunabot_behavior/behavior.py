import rospy
from enum import Enum, auto
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool

import ascent
import find_apriltag
import zones
import lunabot_behavior.excavate as excavate
import deposition
import interrupts
import escape

class States(Enum):
    ASCENT_INIT = auto()
    FIND_TAG = auto()
    TRAVERSAL_MINE = auto()
    EXCAVATE = auto()
    TRENCH = auto()
    ASCENT_MINING = auto()
    TRAVERSAL_BERM = auto()
    ALIGN = auto()
    DEPOSIT = auto()

'''
A class that controls the main behavior of the robot, aiming for a cycle of autonomous mining and berm depositing
Consists of a variety of states, most of which are imported python modules. Publishes robot effort and cmd_vel,
along with the various submodules (which share publishers when possible). For autonomous driving, the class
publishes a boolean state that enables or disables MPC.
'''
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

        self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        self.traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
        self.goal_publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.zone_visual_publisher = rospy.Publisher("/zone_visual", Path, queue_size=1, latch=True)

        self.current_state = States.ASCENT_INIT

        self.start_apriltag: AprilTagDetection = AprilTagDetection()

        self.mining_zone = None
        self.berm_zone = None

        # TODO change to parameters, determine which are needed
        rospy.Subscriber("/sensors", RobotSensors, self.robot_state_callback)
        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/errors", RobotErrors, self.errors_callback)

        rospy.init_node('behavior_node')

    """
    The main method of the class: enables autonomous behavior. Starts up with a few states,
    then goes in a loop of mining/deposition.
    """
    def behavior_loop(self):

        # Initialize all of the modules (before the loop)
        ascent_module = ascent.Ascent(self.effort_publisher)
        find_apriltag_module = find_apriltag.FindAprilTag(self.velocity_publisher)
        excavate_module = excavate.Excavate(self.effort_publisher)

        deposition_module = deposition.Deposition(self.effort_publisher)

        escape_module = escape.Escape(self.velocity_publisher)

        # Startup:

        # disable traversal to begin
        traversal_message = Bool()
        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

        # Raise linear actuators
        rospy.loginfo("State: Ascent")
        self.current_state = States.ASCENT_INIT

        ascent_status = ascent_module.raise_linear_actuators()
        if ascent_status == False: # Robot error
            pass # TODO implement the error functions here

        
        # Spin until we find the start apriltag
        rospy.loginfo("State: Find AprilTag")
        self.current_state = States.FIND_TAG

        apriltag_status = find_apriltag_module.find_apriltag()

        if apriltag_status == "Error": # Robot error
            pass # TODO implement the error functions here
        elif apriltag_status == None: # Could not find apriltag
            pass #TODO pick what to do here
        else:
            self.start_apriltag = apriltag_status

        self.current_state = States.TRAVERSAL_MINE

        # Translate the apriltag into the odom frame
        apriltag_pose_in_odom: PoseStamped = find_apriltag_module.convert_to_odom_frame(self.start_apriltag)

        # Find the mininz/berm zones in the odom frame
        self.mining_zone: zones.Zone = zones.find_mining_zone(apriltag_pose_in_odom)
        self.berm_zone: zones.Zone = zones.find_berm_zone(apriltag_pose_in_odom)

        # Set a goal to the mining zone and publish it
        mining_goal = PoseStamped()
        mining_goal.pose.position.x = self.mining_zone.middle[0]
        mining_goal.pose.position.y = self.mining_zone.middle[1]
        mining_goal.pose.position.z = 0

        mining_goal.header.stamp = rospy.Time.now()
        mining_goal.header.frame_id = "odom"

        self.goal_publisher.publish(mining_goal)

        # This visualizes the given zone as a red square (visible in rviz)
        self.mining_zone.visualize_zone(self.zone_visual_publisher)

        #This loop always running until we end the program
        while(not rospy.is_shutdown()):

            #This loop is running while things are fine. Break out if interrupts
            while (interrupts.check_for_interrupts() == interrupts.Errors.FINE):

                # Drive to the mining area
                if (self.current_state == States.TRAVERSAL_MINE):
                    # Enable traversal (to mining zone)
                    rospy.loginfo("State: Traversal")
                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)
                    
                    # Detect when reached mining zone
                    self.current_state = States.EXCAVATE
                
                # Lower linear actuators and begin spinning excavation
                if (self.current_state == States.EXCAVATE):
                    rospy.loginfo("State: Plunging")

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)

                    excavate_status = excavate_module.excavate()
                    if excavate_status == False:
                        break

                    self.current_state = States.TRENCH

                # Mine, and possibly drive while mining
                if (self.current_state == States.TRENCH):
                    # trench / mine
                    self.current_state = States.ASCENT_MINING

                # Raise linear actuators
                if (self.current_state == States.ASCENT_MINING):
                    rospy.loginfo("State: Ascent")
                    ascent_status = ascent_module.raise_linear_actuators()

                    if ascent_status == False: # Robot error
                        break
                        
                    # Set goal to berm
                    self.current_state = States.TRAVERSAL_BERM

                # Drive to berm area
                if (self.current_state == States.TRAVERSAL_BERM):
                    # Enable traversal (to berm)
                    rospy.loginfo("State: Traversal")

                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)

                    # Detect when reached berm
                    self.current_state = States.ALIGN
            
                # Align with an apriltag at the berm
                if (self.current_state == States.ALIGN):
                    rospy.loginfo("State: Alignment")

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)

                    self.current_state = States.DEPOSIT
                    # Alignment
            
                # Deposit regolith w/ auger
                if (self.current_state == States.DEPOSIT):
                    
                    deposition_status = deposition_module.deposit()

                    if deposition_status == False:
                        break

                    self.current_state = States.TRAVERSAL_MINE

                # Set goal to mining zone
            
            # This block runs when we have an interrupt (some kind of error)
            problem = interrupts.check_for_interrupts()
            if problem == interrupts.Errors.ROS_ENDED:
                #simply exit this loop and the whole program
                break
            elif problem == interrupts.Errors.OVERCURRENT:
                #TODO: what goes here?
                pass
            elif problem == interrupts.Errors.STUCK:
                #if the robot is stuck, unstick it
                escape_module.unstickRobot()
         

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()
    rospy.spin()