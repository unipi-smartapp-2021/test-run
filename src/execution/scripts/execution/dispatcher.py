#!/usr/bin/env python

import math
import rospy
import numpy as np
import carla_msgs.msg
from execution import topics
from execution.PIDController import PIDController
from execution.geometry import quaternion_to_euler
from planning.msg import STP_Data
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Accel

class Dispatcher():
    def __init__(self):
        # Initialize Dispatcher Node
        rospy.init_node('dispatcher', anonymous=True)

        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.current_zorient = 0.0
        self.target_zorient = 0.0

        # Controllers
        self.velocity_control = PIDController(self.current_velocity,
                Kp=6, Kd=0, minv=0, maxv=1)
        self.steering_control = PIDController(self.current_zorient,
                Kp=1.5, Ki=1.2, Kd=1.0, minv=-0.9, maxv=0.9)
        self.braking_control = PIDController(self.current_zorient,
                Kp=3.0, Ki=1.5, Kd=1.0, minv=0.0, maxv=1.0)

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
        self.cmd_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',
                carla_msgs.msg.CarlaEgoVehicleControl)
        # CARLA vehicle information
        self.status_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_status',
                carla_msgs.msg.CarlaEgoVehicleStatus, lambda data: self.update_status(data))
        # self.info_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_info',
        #         carla_msgs.msg.CarlaEgoVehicleInfo, lambda data: self.log_msg(data))

        # STP stub subscription
        self.stp_sub = rospy.Subscriber('stp_data', STP_Data,
                lambda data: self.update_control(data))

        # Initialize actuators states
        self.init_actuators()

    def log_msg(self, data):
        rospy.loginfo(data)

    def init_actuators(self):
        pass

    def update_status(self, data):
        # rospy.loginfo(data)
        # get current linear (forward) velocity
        self.current_velocity = data.velocity
        q = data.orientation
        (x, y, z) = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_zorient = z

    def update_control(self, data):
        # Get target velocity
        self.target_velocity = self.current_velocity + data.dv
        # self.target_zorient = data.psi - np.pi/2.0
        self.target_zorient = 0.0
        rospy.loginfo('current z: {:.3f}\t target z: {:.3}\t psi: {:.3}'.format(self.current_zorient, self.target_zorient, data.psi))

        # Vehicle Control message
        cmd = carla_msgs.msg.CarlaEgoVehicleControl()

        # Control throttle
        cmd.throttle = self.velocity_control.pid_step(self.current_velocity, self.target_velocity)
        # Control steering
        cmd.steer = -self.steering_control.pid_step(self.current_zorient, self.target_zorient)
        # Control braking (if not activating the throttle)
        if data.dv < 0 and cmd.throttle <= 1e-2:
            cmd.brake = self.steering_control.pid_step(self.current_velocity, self.target_velocity)


        self.cmd_pub.publish(cmd)

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
