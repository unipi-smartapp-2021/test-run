#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
import numpy as np
from execution import topics
from std_msgs.msg import String, Float32, Bool

class Dispatcher():
    def __init__(self):
        # Initialize Dispatcher Node
        rospy.init_node('dispatcher', anonymous=True)

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
        self.brake_engage_pub = rospy.Publisher(topics.PPS, Float32)

        # Initialize actuators states
        self.init_actuators()

    def init_actuators(self):
        pass
    
    def spin(self):
        rate = rospy.Rate(10) # 100hz
        while not rospy.is_shutdown():
            signal = np.random.rand()
            rospy.loginfo(signal)

            # actuate

            rate.sleep()


def main():
    dispatcher = Dispatcher()
    dispatcher.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
