#!/usr/bin/env python

import math
import rospy
import numpy as np
import carla_msgs.msg
from execution import topics
from execution.PIDController import PIDController
from execution.geometry import quaternion_to_euler
from tf.transformations import euler_from_quaternion
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
        self.last_cmd = None

        # Controllers
        self.velocity_control = PIDController(0.0,
                Kp=3.0, Ki=1.5, Kd=0.0, minv=0, maxv=1)

        self.steering_control = PIDController(0.0,
                Kp=1.0, Ki=0.1, Kd=0.3, minv=-1.0, maxv=1.0,
                guard=0.1,
                verbose=True)

        self.braking_control = PIDController(0.0,
                Kp=0.1, Ki=0.5, Kd=0.0, minv=0.0, maxv=1.0,
                guard=3.0,
                verbose=False)

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
                lambda data: self.update_command(data))
    
    def update_command(self, data):
        self.last_cmd = data
        # Get target velocity
        self.target_velocity = self.current_velocity + data.dv

        # This fixes the buggy behaviour of the STP at the start
        if self.current_velocity <= 1e-2:
            data.dt = 0.0

        self.target_zorient = -data.psi + data.dt
        # self.target_zorient = 0.0

    def update_status(self, data):
        # get current linear (forward) velocity
        self.current_velocity = data.velocity
        q = data.orientation
        (x, y, z) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_zorient = z

    def update_control(self, data):
        rospy.loginfo('current dv: {:.3f}'.format(data.dv))
        rospy.loginfo('current psi: {:.3f}\t target dt: {:.3}'.format(data.psi, data.dt))
        rospy.loginfo('current z: {:.3f}\t target z: {:.3}'.format(self.current_zorient, self.target_zorient))
        rospy.loginfo('current v: {:.3f}\t target v: {:.3}'.format(self.current_velocity, self.target_velocity))
        # Vehicle Control message
        cmd = carla_msgs.msg.CarlaEgoVehicleControl()

        # Control throttle
        cmd.throttle = self.velocity_control.pid_step(self.current_velocity, self.target_velocity)
        # Control steering
        cmd.steer = -self.steering_control.pid_step(self.current_zorient, self.target_zorient)
        # Control braking (if not activating the throttle)
        brake = 1 - self.braking_control.pid_step(self.current_velocity, self.target_velocity)
        if data.dv < 0 and cmd.throttle <= 1e-2:
            cmd.brake = brake

        # Deactivate steering when braking
        # if cmd.brake > 0.0 or self.target_velocity < 1.0:
        #     cmd.steer = 0.0

        rospy.loginfo('current brake: {:.3f}'.format(cmd.brake))
        rospy.loginfo('current steer: {:.3f}'.format(cmd.steer))
        rospy.loginfo('current throttle: {:.3f}'.format(cmd.throttle))
        rospy.loginfo(10*'-')
        self.cmd_pub.publish(cmd)

    def spin(self):
        rate = rospy.Rate(20) # 100hz
        while not rospy.is_shutdown():
            if self.last_cmd:
                self.update_control(self.last_cmd)
            rate.sleep()


def main():
    dispatcher = Dispatcher()
    dispatcher.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
