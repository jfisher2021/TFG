#!/usr/bin/env python3
"""
@file tts_client.py
@brief ROS 2 client node for testing Text-To-Speech service.

This script defines a simple ROS 2 client that calls the TTS service
with a test message. Useful for testing the TTS service.

@details
- The node creates a client for `/tts_service` (TextToSpeech).
- Sends a request with text "HOLA MUNDO" (default test message).
- Waits for the service response and prints the result.
- Uses synchronous service call (blocking until audio finishes playing).

@node_name gtts_srv_client_node

@services
- Used:
    - `/tts_service` (my_interfaces/srv/TextToSpeech): Triggers TTS.
      - Request: 
          - text (string): Text to convert to speech
      - Response: 
          - success (bool): Whether TTS succeeded
          - debug (string): Error message if failed

@dependencies
- rclpy: For ROS 2 node implementation.
- my_interfaces.srv.TextToSpeech: Custom service definition.

@usage
- Run the client (assumes TTS service is already running):
    ```
    ros2 run speech_services tts_client
    ```
- The client will synthesize and play "HOLA MUNDO".
- Modify the `text` variable in main() to test different messages.

@note
- The TTS service must be running before calling this client.
- This is a test/example client, not meant for production use.
- Audio playback is blocking (waits until audio finishes).

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
from rclpy.duration import Duration
from my_interfaces.srv import TextToSpeech  

# string text
# ---
# bool success
# string debug

class GTTSClientAsync(Node):

    def __init__(self):
        super().__init__('gtts_srv_client_node')
        self.cli = self.create_client(TextToSpeech, 'tts_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TextToSpeech.Request()

    def send_request(self, text):
        self.req.text = text
        self.future = self.cli.call_async(self.req)
        print("spin_until_future_complete")
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    gtts_client = GTTSClientAsync()
    
    text="HOLA MUNDO";
    response = gtts_client.send_request(text)
    print(response.debug)
    gtts_client.get_logger().info(
        'Request complete')


    gtts_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()