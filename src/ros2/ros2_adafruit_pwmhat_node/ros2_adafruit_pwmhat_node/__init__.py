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

import Adafruit_PCA9685
from ros2_adafruit_pwmhat_msgs.srv import BasicPWMchannels

import rclpy
from rclpy.node import Node

class ROS2_Adafruit_pwmhat_node(Node):

    def __init__(self):
        super().__init__('ros2_adafruit_pwmhat_node')
        self.srv = self.create_service(BasicPWMchannels, 'basicpwmchannels', self.basicpwmchannels_callback)
        self.pwm = Adafruit_PCA9685.PCA9685()
        print('sys.path=')
        for path in sys.path:
            print(path)

    def basicpwmchannels_callback(self, request, response):
        self.get_logger().info('Incoming request: chan=%d pulse_length=%d' % (request.chan, request.pulse_length))
        self.pwm.set_pwm(request.chan, 0, request.pulse_length)
        
        response.error = 0

        return response


def main(args=None):
    rclpy.init(args=args)

    pwmService = ROS2_Adafruit_pwmhat_node()

    rclpy.spin(pwmService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
