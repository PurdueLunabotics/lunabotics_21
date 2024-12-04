#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Joy
from lunabot_msgs.msg import RobotEffort, RobotErrors

from enum import Enum
import numpy as np

"""
.buttons   (1 is pressed, 0 not)
0 A
1 B
2 X
3 Y
4 LB
5 RB
6 back (view button)
7 start
8 power
9 button stick left
10 button stick right

.axes
0 left stick l/r     (left = 1.0, right = -1.0)
1 left stick u/d     (up = 1.0, down = -1.0)
2 Left Trigger       (Not pressed = 1.0, Fully depressed = -1.0)
3 right stick l/r    (left = 1.0, right = -1.0)
4 right stick u/d    (up = 1.0, down = -1.0)
5 Right Trigger      (Not pressed = 1.0, Fully depressed = -1.0)
6 dpad (arrows) l/r  (left = 1.0, right = -1.0)
7 dpad (arrows) u/d  (up = 1.0, down = -1.0)
"""

# Take an input from -1 to 1, and convert it to an 8 bit int from -127 to 127
def constrain(joy_in: float):
    return np.int8(np.clip(joy_in, -1, 1) * 127)


# Take an input from -1 to 1, and convert it to an 32 bit int from -5000 to 5000
def constrain_RPM(joy_in: float, max_speed):
    return np.int32(np.clip(joy_in, -1, 1) * max_speed)

class Buttons(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    POWER = 8
    L_STICK = 9
    R_STICK = 10

class Axes(Enum):
    L_STICK_HORIZONTAL = 0
    L_STICK_VERTICAL = 1
    LEFT_TRIGGER = 2
    R_STICK_HORIZONTAL = 3
    R_STICK_VERTICAL = 4
    RIGHT_TRIGGER = 5
    DPAD_HORIZONTAL = 6
    DPAD_VERTICAL = 7


class ManualController:
    """
    Manual Controller
    Publishes an effort message to control the robot.
    Control Scheme:
    (tank drive)
    - Left stick: left wheels
    - Right stick: right wheels
    - Right trigger Excavation forwards (pick up dirt)
    - Left trigger: Excavation backwards
    - D-pad up/down: Linear actuator control (move excavation system up/down)
    - Y button: Latch/unlatch excavation speed
        - When latched, the excavation speed will stay at its last speed until unlatched
    - X button: Switch between forwards and backwards driving
        - Forwards is defined as leading with excavation
    - B button: Deposition (spin auger)
    - Start button: Stop the robot while held
    """

    def __init__(self):
        self.joy_subscriber = rospy.Subscriber("joy0", Joy, self.joy_callback)
        self.effort_publisher = rospy.Publisher("effort", RobotEffort, queue_size=1, latch=True)
        self.effort_msg = RobotEffort()

        self.error_msg = RobotErrors()
        self.error_subscriber = rospy.Subscriber("errors", RobotErrors, self.error_callback)
        self.error_pub = rospy.Publisher("errors", RobotErrors, queue_size=1, latch=True)

        self.last_joy = Joy()
        self.last_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        self.driving_mode = "Forwards"
        
        self._max_speed = rospy.get_param("~max_speed", 2.0) * 60 / 2 / np.pi # In rad/s converted to RPM
        self.drive_speed_modifier = 1
        self.slow_drive_speed = 0.5
        self.fast_drive_speed = 1

        self.latched_excavation_speed = 0
        self.excavation_is_latched = False

        self.DEPOSITION_SPEED = 3000 #TODO RJN - this speed
        self.ACTUATE_SPEED = 0.8 # percentage of max power

        self.publish = True

        self.stop()

    def error_callback(self, error_msg: RobotErrors):
        self.error_msg = error_msg

    def publish_manual_stop(self):
        self.error_msg.manual_stop = True
        self.error_pub.publish(self.error_msg)

    def unpublish_manual_stop(self):
        self.error_msg.manual_stop = False
        self.error_pub.publish(self.error_msg)

    def joy_callback(self, joy):
        # X button: Switch between driving forwards and backwards'

        if joy.buttons[Buttons.X.value] == 1 and self.last_joy.buttons[Buttons.X.value] == 0:
            if (self.driving_mode == "Forwards"):
                self.driving_mode = "Backwards"
            else:
                self.driving_mode = "Forwards"

            rospy.loginfo(f"Driving Direction: {self.driving_mode}")

        if joy.buttons[Buttons.LB.value] == 1 and self.last_joy.buttons[Buttons.LB.value] == 0:
            if (self.drive_speed_modifier <= self.slow_drive_speed):
                self.drive_speed_modifier = self.fast_drive_speed
            else:
                self.drive_speed_modifier = self.slow_drive_speed-0.0001

            rospy.loginfo(f"Driving Speed: {self.drive_speed_modifier}")

        if joy.buttons[Buttons.RB.value] == 1:
            self.publish = False
        else:
            self.publish = True

        # Start button: Stop the robot (Pause)
        if joy.buttons[Buttons.START.value] == 1:
            self.stop()
            self.publish_manual_stop()
            rospy.loginfo("Manual Control: Stopped")
        else:
            self.unpublish_manual_stop()
            effort_msg = RobotEffort()

            effort_msg.left_drive = 0
            effort_msg.right_drive = 0

            effort_msg.excavate = 0

            # Set the drive effort to the left and right stick vertical axes (Tank Drive)
            if self.driving_mode == "Forwards":
                effort_msg.left_drive = constrain_RPM(joy.axes[Axes.L_STICK_VERTICAL.value], self._max_speed) * self.drive_speed_modifier
                effort_msg.right_drive = constrain_RPM(joy.axes[Axes.R_STICK_VERTICAL.value], self._max_speed) * self.drive_speed_modifier
            else:
                effort_msg.left_drive = -1 * constrain_RPM(joy.axes[Axes.R_STICK_VERTICAL.value], self._max_speed) * self.drive_speed_modifier
                effort_msg.right_drive = -1 * constrain_RPM(joy.axes[Axes.L_STICK_VERTICAL.value], self._max_speed) * self.drive_speed_modifier

            effort_msg.left_drive = int(effort_msg.left_drive)
            effort_msg.right_drive = int(effort_msg.right_drive)


            # If not latched, use the trigger axis to control the excavation speed. Otherwise, use the latched speed
            if self.excavation_is_latched:
                effort_msg.excavate = self.latched_excavation_speed
            else:
                # Change the range of the trigger axis from [1, -1] to [0, 1]

                # if the value is exactly 0, the joystick has not been properly started, so reset excavation to not move

                if (joy.axes[Axes.RIGHT_TRIGGER.value] == 0):
                    right_trigger_axis_normalized = 0
                    #print("normR")
                else:
                    right_trigger_axis_normalized = (-joy.axes[Axes.RIGHT_TRIGGER.value] + 1) / 2

                if (joy.axes[Axes.LEFT_TRIGGER.value] == 0):
                    left_trigger_axis_normalized = 0
                    #print("norm")
                else:
                    left_trigger_axis_normalized = (-joy.axes[Axes.LEFT_TRIGGER.value] + 1) / 2

                # Take priority for right trigger. If it is nearly zero, use the left trigger instead
                if (right_trigger_axis_normalized <= 0.01):
                    effort_msg.excavate = -1 * constrain_RPM(left_trigger_axis_normalized, 1000)
                else:
                    effort_msg.excavate = constrain_RPM(right_trigger_axis_normalized, 1000)

            # Y button: latch excavation speed. This keeps excavation at the same speed until the latch is released
            if joy.buttons[Buttons.Y.value] == 1 and self.last_joy.buttons[Buttons.Y.value] == 0:  
                if (self.excavation_is_latched):
                    self.excavation_is_latched = False
                    rospy.loginfo("Excavation Released")
                else:
                    self.excavation_is_latched = True
                    self.latched_excavation_speed = effort_msg.excavate
                    rospy.loginfo(f"Excavation Latched at {self.latched_excavation_speed}")

            # Dpad up/down - control linear actuators
            effort_msg.lin_act = int(constrain(joy.axes[Axes.DPAD_VERTICAL.value]) * self.ACTUATE_SPEED)

            # Deposition- B to go, view/select/back to move backwards
            if (joy.buttons[Buttons.B.value] == 1):
                effort_msg.deposit = self.DEPOSITION_SPEED
            elif (joy.buttons[Buttons.BACK.value] == 1):
                effort_msg.deposit = -1 * self.DEPOSITION_SPEED

            self.effort_msg = effort_msg
            self.last_joy = joy

    def loop(self):
        if self.publish:
            self.effort_publisher.publish(self.effort_msg)

    def stop(self):
        self.effort_msg.left_drive = 0
        self.effort_msg.right_drive = 0
        self.effort_msg.excavate = 0
        self.effort_msg.lin_act = 0
        self.effort_msg.deposit = 0

        self._exc_latch_val = 0
        self._exc_latch = True

        self.effort_publisher.publish(self.effort_msg)


if __name__ == "__main__":
    rospy.init_node("manual_controller_node")

    man_ctrl = ManualController()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if not rospy.get_param("autonomy"):
            man_ctrl.loop()
        rate.sleep()
