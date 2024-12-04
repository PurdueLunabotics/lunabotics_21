import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors
from std_msgs.msg import Int8, Int32
import interrupts

import time

class Plunge:
    '''
    This is a transition state that spins excavation and lowers the linear actuators to begin mining.
    Outdated: Use excavate.py which has plunge/trench
    '''
    
    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, excavation_publisher: rospy.Publisher = None, lin_act_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if excavation_publisher is None:
            self.excavation_publisher = rospy.Publisher("/excavate", Int32, queue_size=1, latch=True)
            rospy.init_node('plunge_node')
        else:
            self.excavation_publisher = excavation_publisher

        if lin_act_publisher is None:
            self.lin_act_publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
        else:
            self.lin_act_publisher = lin_act_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)

        self.exc_stuck = False
        self.stuck_time = rospy.get_time()

        # TODO: check and test this value
        self.LOWERING_TIME = 23 #In seconds, how long it takes to lower the linear actuators 90% of the way

        self.EXCAVATION_SPEED = 1000 #RPM
        self.LIN_ACT_SPEED = -110

        self.is_sim = rospy.get_param("is_sim")

    def check_exc_stuck(self, exc_msg):
        # check if trying to go somewhere
        if exc_msg >= 300:
            # check if not going anywhere
            if self.robot_sensors.exc_vel <= 0.1:
                self.exc_stuck = True

    def plunge(self):
        """
        Spin excavation at full speed, lower linear actuators 90% to begin mining, and then reduce speed to 25%
        """

        if (self.is_sim):
            rospy.loginfo("Plunge: would plunge")
            time.sleep(2)
            return True

        time.sleep(0.1)

        excavation_message = Int32()
        excavation_message.data = self.EXCAVATION_SPEED

        lin_act_message = Int8()
        lin_act_message.data = self.LIN_ACT_SPEED

        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < self.LOWERING_TIME):

            self.excavation_publisher.publish(excavation_message)
            self.lin_act_publisher.publish(lin_act_message)

            self.check_exc_stuck(excavation_message.data)
            
            if self.exc_stuck:
                pass
                #effort_message.lin_act = 0
                #effort_message.excavate = 0

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False

            self.rate.sleep()

        lin_act_message.data = 0
        excavation_message.data = int(0.25 * self.EXCAVATION_SPEED) #25% speed

        self.lin_act_publisher.publish(lin_act_message)
        self.excavation_publisher.publish(excavation_message)

        return True
    
if __name__ == "__main__":
    plunge = Plunge()
    plunge.plunge()
