# Copyright 2018 Robert Adams
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

import rclpy
from rclpy.node import Node

import Adafruit_PCA9685

from ros2_adafruit_pwmhat_msgs.srv import BasicPWMchannels

class ROS2_Adafruit_pwmhat_node(Node):

    def __init__(self):
        super().__init__('ros2_adafruit_pwmhat_node')
        self.srv = self.create_service(BasicPWMchannels, 'pwmhat/pulselength', self.basicpwmchannels_callback)
        self.pwm = Adafruit_PCA9685.PCA9685()
        # frequency can be from 40 to 1000
        self.pwm_frequency = 60
        self.pwm.set_pwm_freq(self.pwm_frequency)

    def basicpwmchannels_callback(self, request, response):
        # self.get_logger().info(f"Incoming request: chan={request.chan} pulse_length={request.pulse_length}")
        self.get_logger().info("Incoming request: chan=%s pulse_length=%s" % (request.chan, request.pulse_length) )

        self.set_pwm_by_pulse_length(request.chan, request.pulse_length)
        
        response.error = 0

        return response

    def set_pwm_by_pulse_length(self, pin, pulse_length):
        # turn specified PWM channel on for the given pulse_length in microseconds
        pulse_scaler = 1000000              # microseconds
        pulse_scaler //= self.pwm_frequency # microseconds per cycle
        pulse_scaler //= 4096               # microseconds per PWM count
        conv_pulse_length = int((pulse_length * 1000) / pulse_scaler)
        conv_pin = int(pin)
        # pwm.set_pwm(channel, onTickTime, offTickTime) where on/off are within 0..4095
        self.pwm.set_pwm(conv_pin, 0, conv_pulse_length)


def main(args=None):
    rclpy.init(args=args)

    pwmService = ROS2_Adafruit_pwmhat_node()

    rclpy.spin(pwmService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
