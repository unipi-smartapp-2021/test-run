#!/usr/bin/env python

import rospy
import numpy as np
import carla_msgs.msg
from execution import topics
from execution.AccelerationController import AccelerationController
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Accel


class Dispatcher():
    def __init__(self):
        # Initialize Dispatcher Node
        rospy.init_node('dispatcher', anonymous=True)

        self.acceleration_control = AccelerationController(0.0, Kp=6, Kd=0)

        # Steering wheel
        self.steer_enable_pub = rospy.Publisher(topics.STEER_ENABLE, Bool)
        self.steer_pub = rospy.Publisher(
                topics.STEER_DESIRED_WHEEL_ANGLE,
                Float32
                )
        # self.steer_current_angle_sub = ...

        # Brake
        self.brake_homing_pub = rospy.Publisher(topics.BRAKE_HOMING, Bool)
        self.brake_engage_pub = rospy.Publisher(topics.BRAKE_ENGAGE, Bool)
        self.brake_des_percent_pub = rospy.Publisher(
                topics.BRAKE_DESIRED_PERCENT,
                Float32
                )

        # Clutch
        self.clutch_enable_pub = rospy.Publisher(topics.CLUTCH_ENABLE, Bool)
        self.clutch_engage_pub = rospy.Publisher(topics.CLUTCH_ENGAGE, Bool)

        # PPS (Pedal Position Sensor)
        self.pps_pub = rospy.Publisher(topics.PPS, Float32)

        self.pps_sub = rospy.Subscriber(topics.PPS, Float32, lambda data: self.log_msg(data))

        # CARLA vehicle control commands
        self.cmd_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', carla_msgs.msg.CarlaEgoVehicleControl)
        # CARLA vehicle information
        self.status_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_status', carla_msgs.msg.CarlaEgoVehicleStatus, lambda data: self.update_throttle(data))
        self.info_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_info', carla_msgs.msg.CarlaEgoVehicleInfo, lambda data: self.log_msg(data))

        # Initialize actuators states
        self.init_actuators()

    def log_msg(self, data):
        rospy.loginfo(data)

    def init_actuators(self):
        pass

    def update_throttle(self, data):
        rospy.loginfo(data)
        # Get target acceleration
        target_acc = self.get_target_acceleration()

        # get current linear (forward) acceleration
        current = data.acceleration.linear
        # compute acceleration modulus
        current = np.sqrt(current.x**2 + current.y**2)

        # Vehicle Control message
        cmd = carla_msgs.msg.CarlaEgoVehicleControl()
        cmd.throttle = self.acceleration_control.pid_step(current, target_acc)
        self.cmd_pub.publish(cmd)

    def get_target_acceleration(self):
        # target acceleration stub
        return 2.0 # m/s^2
    
    def spin(self):
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    dispatcher = Dispatcher()
    dispatcher.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
