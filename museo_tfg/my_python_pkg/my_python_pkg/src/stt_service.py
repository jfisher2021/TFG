"""
@file stt_service.py
@brief ROS 2 service node for Speech-To-Text using Whisper.

This script defines a ROS 2 service node that records audio, transcribes it using 
the Whisper model, and returns the transcription as a service response.

@details
- The node provides a service `/stt_service` (SpeechToText).
- When called, it records audio for a configurable duration, processes it, 
  and transcribes it using Whisper with Spanish language forced.
- The transcription is returned directly in the service response.
- The Whisper model is loaded once at initialization for better performance.

@node_name stt_service_node

@services
- Provided:
    - `/stt_service` (my_interfaces/srv/SpeechToText): Records audio and returns transcription.
      - Request: (empty, just triggers recording)
      - Response: 
          - success (bool): Whether transcription succeeded
          - text (string): The transcribed text
          - debug (string): Error traceback if failed

@dependencies
- whisper: For audio transcription (Spanish language forced).
- sounddevice: For audio recording.
- scipy.io.wavfile: For saving audio files.
- rclpy: For ROS 2 node implementation.
- my_interfaces.srv.SpeechToText: Custom service definition.

@configuration
- The configuration is loaded from `my_python_pkg.src.user_config.UserConfig` and includes:
    - `duration`: Duration of audio recording in seconds.
    - `volume_gain_multiplier`: Multiplier for adjusting audio volume.
- Whisper model: 'small' (configurable in code)
- Language: 'es' (Spanish, forced)

@usage
- Run the node:
    ```
    ros2 run my_python_pkg stt_service
    ```
- Test the service:
    ```
    ros2 service call /stt_service my_interfaces/srv/SpeechToText
    ```

@note
- Ensure that the Whisper model is installed and accessible.
- The audio file is temporarily saved in `/tmp/user_audio_input.wav`.
- The virtual environment at `/home/jfisherr/cuarto/2c/plansis/plansys_ws/venv_plansys2` 
  must contain the whisper package.

@author Jonathan Fisher @j.fisher.2021
@license Apache License, Version 2.0
"""

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
from my_interfaces.srv import SpeechToText

# Global Configuration
from my_python_pkg.src.user_config import UserConfig
import traceback

config = UserConfig()


class AudioInput(Node):
    def __init__(self):
        super().__init__("stt_service_node")

        self.srv = self.create_service(SpeechToText, "stt_service", self.gstt_callback)
        
        # Cargar modelo Whisper UNA SOLA VEZ al inicializar
        self.get_logger().info('Cargando modelo Whisper...')
        self.whisper_model = whisper.load_model("small")
        self.get_logger().info('GSTTService con Whisper API inicializado')
        
        # Archivo de audio local
        self.audio_file = "/tmp/user_audio_input.wav"


    def gstt_callback(self, request, response):
        """Graba audio y lo transcribe usando Whisper."""
        try:
            duration = config.duration
            sample_rate = 44100
            volume_gain_multiplier = config.volume_gain_multiplier
            
            # Grabar audio
            self.get_logger().info("Grabando audio...")
            audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype="float32"
            )
            sd.wait()

            # Ajustar volumen
            audio_data *= volume_gain_multiplier

            # Guardar archivo de audio
            write(self.audio_file, sample_rate, (audio_data * 32767).astype("int16"))
            self.get_logger().info("Grabación completada.")

            # Transcribir con Whisper 
            self.get_logger().info("Transcribiendo con Whisper")
            result = self.whisper_model.transcribe(self.audio_file, language='es')

            transcript_text = result["text"].strip()

        except Exception as e:
            tb = traceback.format_exc()
            self.get_logger().error(f"Error en gstt_callback: {e}\n{tb}")
            # Informar el error en la respuesta del servicio
            response.success = False
            response.text = ""
            response.debug = tb
            return response
        self.get_logger().info(f"Transcripción: {transcript_text}")

        # Retornar la transcripción directamente en la respuesta del servicio
        response.success = True
        response.text = transcript_text
        return response



def main(args=None):
    rclpy.init(args=args)
    audio_input = AudioInput()
    rclpy.spin(audio_input)
    audio_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
