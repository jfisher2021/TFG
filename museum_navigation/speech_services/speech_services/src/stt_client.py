#!/usr/bin/env python3
"""
@file stt_client.py
@brief ROS 2 client node for testing Speech-To-Text service.

This script defines a simple ROS 2 client that calls the STT service
and prints the transcription result. Useful for testing the STT service.

@details
- The node creates a client for `/stt_service` (SpeechToText).
- Sends a request to trigger audio recording and transcription.
- Waits for the service response and prints the transcribed text.
- Uses synchronous service call (blocking).

@node_name stt_srv_client_node

@services
- Used:
    - `/stt_service` (my_interfaces/srv/SpeechToText): Triggers STT.
      - Request: (empty)
      - Response: 
          - success (bool): Whether transcription succeeded
          - text (string): The transcribed text
          - debug (string): Error traceback if failed

@dependencies
- rclpy: For ROS 2 node implementation.
- my_interfaces.srv.SpeechToText: Custom service definition.

@usage
- Run the client (assumes STT service is already running):
    ```
    ros2 run speech_services stt_client
    ```
- The client will trigger recording, wait for transcription, and print result.

@note
- The STT service must be running before calling this client.
- This is a test/example client, not meant for production use.

@author Jonathan Fisher
@license Apache License, Version 2.0
"""

# Copyright 2024 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import SpeechToText

# ---
# bool success
# string text
# string debug

class GSTTClientAsync(Node):

    def __init__(self):
        super().__init__('stt_srv_client_node')
        self.cli = self.create_client(SpeechToText, 'stt_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpeechToText.Request()

    def send_request(self, cmd):
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    gstt_client = GSTTClientAsync()
    start=True;
    response = gstt_client.send_request(start)
    print(response)
    gstt_client.get_logger().info(
        'Request complete')


    gstt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()