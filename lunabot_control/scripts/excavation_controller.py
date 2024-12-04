#!/usr/bin/env python3
import numpy as np
import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors


class LeakyBucket:
    def __init__(self, condition_fn, inc=100, dec=10, capacity=1000):
        self.condition_fn = condition_fn
        self.inc_ = inc
        self.dec_ = dec
        self.capacity_ = capacity
        self.overflow_ = False
        self.count_ = 0

    def is_overflow(self):
        if self.condition_fn():
            self.count_ += self.inc_
        else:
            self.count_ = max(0, self.count_ - self.dec_)
        if self.count_ >= self.capacity_:
            self.overflow_ = True
        else:
            self.overflow_ = False
        return self.overflow_


class ExcavationController:
    FORWARD_EXCAVATE = -5000
    DOWN_LEAD_SCREW = -0.2
    UP_LEAD_SCREW = 0.7

    valid_exc_current_range = (6000, 17400)
    valid_lead_screw_current_range = (6000, 20000)

    def __init__(self):
        self.exc_curr = None
        self.lead_screw_curr = None
        self.retract = False

        self.exc_rpm = self.FORWARD_EXCAVATE
        self.lead_screw_voltage = self.DOWN_LEAD_SCREW

        self.exc_leaky_bucket = LeakyBucket(self.is_exc_stuck, dec=20)
        self.lead_screw_leaky_bucket = LeakyBucket(
            self.is_lead_screw_invalid, dec=1000, inc=100
        )

        self._effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)
        self._state_sub = rospy.Subscriber(
            "/sensors", RobotSensors, self._robot_state_cb
        )

        self.max_exc_percent = rospy.get_param("~max_exc_percent", 1)
        self.max_lead_screw_percent = rospy.get_param("~max_lead_screw_percent", 1)
        self.hz = rospy.get_param("~hz", 50)

        rospy.on_shutdown(self.shutdown_hook)
        self.rate = rospy.Rate(self.hz)

        while not rospy.is_shutdown():
            if self.exc_curr is not None and self.lead_screw_curr is not None:
                exc_overflow = self.exc_leaky_bucket.is_overflow()
                lead_screw_overflow = self.lead_screw_leaky_bucket.is_overflow()
                print("max extend", self.lead_screw_max_extend())
                print("max retract", self.lead_screw_max_retract())

                if not self.retract:
                    if exc_overflow:
                        print("EXC OVERFLOW, MOVING UP")
                        self.lead_screw_voltage = self.UP_LEAD_SCREW
                    else:
                        self.lead_screw_voltage = self.DOWN_LEAD_SCREW
                else:
                    self.exc_rpm = 0

                if lead_screw_overflow:
                    if self.lead_screw_max_extend():
                        print("LEAD_SCREW_OVERFLOW, RETRACTING")
                        print("max_retract", self.lead_screw_max_retract())
                        self.retract = True
                        self.lead_screw_voltage = self.UP_LEAD_SCREW
                    elif self.lead_screw_max_retract():
                        print("LEAD_SCREW_OVERFLOW, STOPPING")
                        self.stop()
                        self.retract = False
                        break
                else:
                    print("LEAD SCREW OVERFLOW RELEASED")
                self.publish_effort()
            self.rate.sleep()

    def _robot_state_cb(self, msg):
        self.exc_curr = msg.exc_curr
        self.lead_screw_curr = msg.lead_screw_curr

    def is_exc_stuck(self):
        return (
            self.exc_curr < self.valid_exc_current_range[0]
            or self.exc_curr > self.valid_exc_current_range[1]
        )

    def lead_screw_max_retract(self):
        return self.lead_screw_curr < self.valid_lead_screw_current_range[0]

    def lead_screw_max_extend(self):
        return self.lead_screw_curr > self.valid_lead_screw_current_range[1]

    def is_lead_screw_invalid(self):
        return self.lead_screw_max_retract() or self.lead_screw_max_extend()

    def publish_effort(self):
        effort_msg = RobotEffort()
        effort_msg.lead_screw = self.constrain(
            self.lead_screw_voltage, max_percent=self.max_lead_screw_percent
        )
        effort_msg.excavate = self.constrain_RPM(
            self.exc_rpm, max_percent=self.max_exc_percent
        )
        self._effort_pub.publish(effort_msg)

    def constrain(self, val, max_percent):
        val = np.clip(-1, val, 1)  # Clipping speed to not go over 100%
        return np.int32(val * 127 * max_percent)
    
    def constrain_RPM(self, val, max_percent):
        val = np.clip(-1, val, 1)  # Clipping speed to not go over 100%
        return np.int32(val * 5000 * max_percent) #TODO RJN - check this threshold


    def stop(self):
        effort_msg = RobotEffort()
        self._effort_pub.publish(effort_msg)

    def shutdown_hook(self):
        self.stop()
        rospy.logerr("stopping excavation control")


if __name__ == "__main__":
    rospy.init_node("excavation_controller")
    controller = ExcavationController()
