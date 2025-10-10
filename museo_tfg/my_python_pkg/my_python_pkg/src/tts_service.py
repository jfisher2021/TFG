#!/usr/bin/env python3

# Copyright 2024 Antonio Bono
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
venv_site = "/home/jfisherr/cuarto/2c/plansis/plansys_ws/venv_plansys2/lib/python3.12/site-packages"

# Asegúrate de que esté en sys.path
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)
print(sys.executable)
import os
from google.cloud import texttospeech
import rclpy
from rclpy.node import Node
from my_interfaces.srv import TextToSpeech  
import subprocess

class GTTSService(Node):
    def __init__(self):
        super().__init__("gtts_srv_node")

        self.client = texttospeech.TextToSpeechClient()

        self.voice = texttospeech.VoiceSelectionParams(
            language_code="es-ES", name="es-ES-Standard-H"
        )

        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.OGG_OPUS,
            pitch = -2,
            speaking_rate= 1.2 
        )

        self.srv = self.create_service(TextToSpeech, "tts_service", self.gtts_callback)

        self.get_logger().info("✅ GTTSService Server initialized.")

    def gtts_callback(self, request, response):
        reqText = request.text.strip()

        if not reqText:
            response.success = False
            response.debug = "⚠️ Texto vacío para convertir"
            self.get_logger().warn(response.debug)
            return response

        synthesis_input = texttospeech.SynthesisInput(text=reqText)
        tts_response = self.client.synthesize_speech(
            input=synthesis_input, voice=self.voice, audio_config=self.audio_config
        )

        audio_path = "/tmp/output.ogg"
        with open(audio_path, "wb") as out:
            out.write(tts_response.audio_content)

        self.get_logger().info(f"🔊 Reproduciendo audio generado: {audio_path}")

        # ⚠️ Reproducir usando ffplay
        # Reproduce y espera a que termine
        try:
            subprocess.run(
                ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", audio_path],
                check=True
            )
        except subprocess.CalledProcessError as e:
            response.success = False
            response.debug = f"Error en reproducción: {e}"
            self.get_logger().error(response.debug)
            return response

        response.success = True
        response.debug = "✅ TTS completado correctamente"
        self.get_logger().info(response.debug)
        return response


def main():
    rclpy.init()
    tts_service = GTTSService()
    rclpy.spin(tts_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
