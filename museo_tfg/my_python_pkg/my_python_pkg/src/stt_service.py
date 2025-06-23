"""
@file llm_audio_input.py
@brief ROS 2 node for audio input processing and transcription using Whisper.

This script defines a ROS 2 node that listens for a specific state, records audio, 
transcribes it using the Whisper model, and publishes the transcription to a ROS 2 topic. 
It also includes functionality for adjusting audio volume and handling state transitions.

@details
- The node subscribes to the `/llm_state` topic to listen for state changes.
- When the state is "listening", it records audio, processes it, and publishes the transcription.
- The transcription is published to the `/llm_input_audio_to_text` topic.
- The node uses the Whisper library for audio transcription and supports configurable parameters 
    such as recording duration and volume gain multiplier.

@node_name llm_audio_input

@topics
- Subscribed:
    - `/llm_state` (std_msgs/msg/String): Listens for state changes to trigger actions.
- Published:
    - `/llm_initialization_state` (std_msgs/msg/String): Indicates the initialization state of the node.
    - `/llm_state` (std_msgs/msg/String): Publishes the current state of the node.
    - `/llm_input_audio_to_text` (std_msgs/msg/String): Publishes the transcribed audio text.

@dependencies
- whisper: For audio transcription.
- sounddevice: For audio recording.
- scipy.io.wavfile: For saving audio files.
- rclpy: For ROS 2 node implementation.
- std_msgs.msg.String: For ROS 2 message types.

@configuration
- The configuration is loaded from `llm_config.user_config.UserConfig` and includes:
    - `duration`: Duration of audio recording in seconds.
    - `volume_gain_multiplier`: Multiplier for adjusting audio volume.

@usage
- Run the node:
    ```
    ros2 run llm_input llm_audio_input
    ```
- Test the node:
    ```
    ros2 topic echo /llm_input_audio_to_text
    ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
    ```

@note
- Ensure that the Whisper model is installed and accessible.
- The audio file is temporarily saved in `/tmp/user_audio_input.wav`.

@author Herman Ye @Auromix
@Modified by Jonathan Fisher @j.fisher.2021
@copyright 2023 Herman Ye @Auromix
@license Apache License, Version 2.0
"""
# # -*- coding: utf-8 -*-
# # flake8: noqa
# #
# # Copyright 2023 Herman Ye @Auromix
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #
# # Description:
# #
# # Node test Method:
# # ros2 run llm_input llm_audio_input
# # ros2 topic echo /llm_input_audio_to_text
# # ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
# #
# # Author: Herman Ye @Auromix


import datetime
import json
import time
import sys
# Forzar a usar el entorno virtual si no se está usando ya

# Ruta a tu venv
venv_site = "/home/jfisherr/cuarto/2c/plansis/plansys_ws/venv_plansys2/lib/python3.12/site-packages"

# Asegúrate de que esté en sys.path
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)
print(sys.executable)
import whisper  # Importamos Whisper para transcribir el audio
import sounddevice as sd
from scipy.io.wavfile import write
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

# Global Configuration
from my_python_pkg.src.user_config import UserConfig

config = UserConfig()


class AudioInput(Node):
    def __init__(self):
        super().__init__("stt_service_node")

        self.srv = self.create_service(SetBool, "stt_service", self.gstt_callback)
        self.get_logger().info('✅ GSTTService con Whisper API inicializado')
        # Archivo de audio local
        self.audio_file = "/tmp/user_audio_input.wav"


    def gstt_callback(self, request, response):
        """Graba audio y lo transcribe usando Whisper."""
        duration = config.duration
        sample_rate = 44100
        volume_gain_multiplier = config.volume_gain_multiplier

        # Paso 1: Grabar audio
        self.get_logger().info("🎤 Grabando audio...")
        audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype="float32"
        )
        sd.wait()

        # Ajustar volumen
        audio_data *= volume_gain_multiplier

        # Guardar archivo de audio
        write(self.audio_file, sample_rate, (audio_data * 32767).astype("int16"))
        self.get_logger().info("✅ Grabación completada.")

        # Paso 2: Transcribir con Whisper
        self.get_logger().info("📝 Transcribiendo con Whisper...")
        model = whisper.load_model("small")  # Puedes usar "base", "small", "medium", "large"
        result = model.transcribe(self.audio_file)

        transcript_text = result["text"].strip()
        self.get_logger().info(f"🗣️ Transcripción: {transcript_text}")

        # Paso 3: Publicar la transcripción en ROS 2
        if not transcript_text:
            self.get_logger().info("⚠️ Entrada vacía, esperando nueva grabación...")
            self.publish_string("listening", self.llm_state_publisher)
        else:
            self.publish_string(transcript_text, self.audio_to_text_publisher)
        
        response.success = True
        response.message = transcript_text
        return response



def main(args=None):
    rclpy.init(args=args)
    audio_input = AudioInput()
    rclpy.spin(audio_input)
    audio_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
