#!/usr/bin/env python3

import rospy
from enum import Enum, auto
import math
import numpy as np

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool, Int8, Int32

import ascent
import find_apriltag
import zones
import excavate
import homing_controller
import deposition
import interrupts
import escape

class States(Enum):
    ASCENT_INIT = auto()
    FIND_TAG = auto()
    TRAVERSAL_MINE = auto()
    PLUNGE = auto()
    TRENCH = auto()
    ASCENT_MINING = auto()
    TRAVERSAL_BERM = auto()
    ALIGN = auto()
    DEPOSIT = auto()

class Behavior:
    '''
    A class that controls the main behavior of the robot, aiming for a cycle of autonomous mining and berm depositing
    Consists of a variety of states, most of which are imported python modules. Publishes robot effort and cmd_vel,
    along with the various submodules (which share publishers when possible). For autonomous driving, the class
    publishes a boolean state that enables or disables MPC.
    '''

    def robot_state_callback(self, msg: RobotSensors):
        self.robot_state = msg

    def errors_callback(self, msg: RobotErrors):
        self.robot_errors = msg

    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg

    def __init__(self):

        rospy.init_node('behavior_node')

        self.robot_state: RobotSensors = RobotSensors()
        self.robot_errors: RobotErrors = RobotErrors()
        self.robot_odom: Odometry = Odometry()

        self.lin_act_publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
        self.left_drive_publisher = rospy.Publisher("/left_drive", Int32, queue_size=1, latch=True)
        self.right_drive_publisher = rospy.Publisher("/right_drive", Int32, queue_size=1, latch=True)
        self.excavate_publisher = rospy.Publisher("/excavate", Int32, queue_size=1, latch=True)
        self.deposition_publisher = rospy.Publisher("/deposition", Int32, queue_size=1, latch=True)

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        self.traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
        self.goal_publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.zone_visual_publisher = rospy.Publisher("/zone_visual", Path, queue_size=1, latch=True)

        self.current_state = States.ASCENT_INIT

        self.start_apriltag: AprilTagDetection = AprilTagDetection()

        self.mining_zone = None
        self.berm_zone = None

        self.rate = rospy.Rate(1) #hz

        self.is_sim = rospy.get_param("/is_sim")

        odom_topic = rospy.get_param("/odom_topic")

        # TODO change to parameters, determine which are needed
        rospy.Subscriber("/sensors", RobotSensors, self.robot_state_callback)
        rospy.Subscriber("/errors", RobotErrors, self.errors_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)


    def is_close_to_goal(self, goal: PoseStamped) -> bool:
        """
        Checks if the robot is close to a given goal
        """

        THRESHOLD = 0.6 # meters

        x = self.robot_odom.pose.pose.position.x
        y = self.robot_odom.pose.pose.position.y

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        distance = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        rospy.logdebug("Behavior: Distance to goal: " + str(distance))

        return distance < THRESHOLD


    def behavior_loop(self):
        """
        The main method of the class: enables autonomous behavior. Starts up with a few states,
        then goes in a loop of mining/deposition.
        """

        # Initialize all of the modules (before the loop)
        ascent_module = ascent.Ascent(self.lin_act_publisher)
        find_apriltag_module = find_apriltag.FindAprilTag(self.velocity_publisher)
        excavation_module = excavate.Excavate(self.excavate_publisher, self.lin_act_publisher, self.velocity_publisher)
        homing_module = homing_controller.HomingController(self.velocity_publisher)
        deposition_module = deposition.Deposition(self.deposition_publisher)

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

        # find_apriltag_module.spin()

        apriltag_status = find_apriltag_module.find_apriltag()

        if apriltag_status == "Error": # Robot error
            pass # TODO implement the error functions here
        elif apriltag_status == None: # Could not find apriltag
            pass #TODO pick what to do here
        else:
            self.start_apriltag = apriltag_status

        # Translate the apriltag into the odom frame
        apriltag_pose_in_odom: PoseStamped = find_apriltag_module.convert_to_odom_frame(self.start_apriltag)

        # Find the mininz/berm zones in the odom frame
        self.mining_zone: zones.Zone = zones.find_mining_zone(apriltag_pose_in_odom, self.is_sim)
        self.berm_zone: zones.Zone = zones.find_berm_zone(apriltag_pose_in_odom, self.is_sim)

        # Set a goal to the mining zone and publish it
        mining_goal = PoseStamped()
        mining_goal.pose.position.x = self.mining_zone.middle[0]
        mining_goal.pose.position.y = self.mining_zone.middle[1]

        offset = zones.calc_offset(-1, 0, apriltag_pose_in_odom, self.is_sim)
        mining_goal.pose.position.x += offset[0]
        mining_goal.pose.position.y += offset[1]

        mining_goal.pose.position.z = 0

        mining_goal.header.stamp = rospy.Time.now()
        mining_goal.header.frame_id = "odom"

        # create a goal for the berm, use the same mining goal
        berm_goal = PoseStamped()
        berm_goal.pose.position.x = mining_goal.pose.position.x
        berm_goal.pose.position.y = mining_goal.pose.position.y

        offset = zones.calc_offset(1, -1.5, apriltag_pose_in_odom, self.is_sim)
        berm_goal.pose.position.x += offset[0]
        berm_goal.pose.position.y += offset[1]

        berm_goal.pose.position.z = 0

        berm_goal.header.stamp = rospy.Time.now()
        berm_goal.header.frame_id = "odom"

        self.current_state = States.TRAVERSAL_MINE

        homing_module.align_to_angle(apriltag_pose_in_odom.pose, np.pi / 2)

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


                    while (not self.is_close_to_goal(mining_goal)):
                        # Wait until close enough
                        self.rate.sleep()
                        if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                            break

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)
                    
                    self.current_state = States.PLUNGE
                
                # Lower linear actuators and begin spinning excavation
                if (self.current_state == States.PLUNGE):
                    rospy.loginfo("State: Plunging")

                    plunge_status = excavation_module.plunge()
                    if plunge_status == False:
                        break

                    self.current_state = States.TRENCH

                # Mine, and possibly drive while mining
                if (self.current_state == States.TRENCH):
                    rospy.loginfo("State: Trenching")
                    
                    trench_status = excavation_module.trench()
                    if trench_status == False:
                        break

                    self.current_state = States.ASCENT_MINING

                # Raise linear actuators
                if (self.current_state == States.ASCENT_MINING):
                    rospy.loginfo("State: Ascent")
                    ascent_status = ascent_module.raise_linear_actuators()

                    if ascent_status == False: # Robot error
                        break
                        
                    # Set a goal to the berm zone and publish it
                    berm_goal.header.stamp = rospy.Time.now()
                    self.goal_publisher.publish(berm_goal)
                    self.berm_zone.visualize_zone(self.zone_visual_publisher)
                    
                    self.current_state = States.TRAVERSAL_BERM

                # Drive to berm area
                if (self.current_state == States.TRAVERSAL_BERM):
                    # Enable traversal (to berm)
                    rospy.loginfo("State: Traversal")

                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)

                    while (not self.is_close_to_goal(berm_goal)):
                        # Wait until close enough
                        self.rate.sleep()
                        if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                            break

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)

                    # Detect when reached berm
                    self.current_state = States.ALIGN
            
                # Align with an apriltag at the berm
                if (self.current_state == States.ALIGN):
                    rospy.loginfo("State: Alignment")

                    homing_module.align_to_angle(apriltag_pose_in_odom.pose, np.pi / 2)

                    alignment_status = homing_module.home()
                    if alignment_status == False:
                        break

                    rospy.loginfo("State: approach")
                    approach_status = homing_module.approach()
                    if approach_status == False:
                        break

                    self.current_state = States.DEPOSIT
            
                # Deposit regolith w/ auger
                if (self.current_state == States.DEPOSIT):
                    rospy.loginfo("State: Deposit")
                    
                    deposition_status = deposition_module.deposit()

                    if deposition_status == False:
                        break

                    # Set a goal to the mining zone and publish it
                    mining_goal.header.stamp = rospy.Time.now()
                    self.goal_publisher.publish(mining_goal)
                    self.mining_zone.visualize_zone(self.zone_visual_publisher)

                    self.current_state = States.TRAVERSAL_MINE

                self.rate.sleep()
                
            
            # This block runs when we have an interrupt (some kind of error)
            problem = interrupts.check_for_interrupts()
            if problem == interrupts.Errors.ROS_ENDED:
                #simply exit this loop and the whole program
                break
            elif problem == interrupts.Errors.MANUAL_STOP:
                # also exit the loop/stop autonomy
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
