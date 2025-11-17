# Fichero creado para hacer pruebas con el fin de evaluar los distintos modelos
# de LLMs. Se intenta ver si poniendo un modelo validador, mejora el plan y se puede evaluar.
# Está pensado para iteración rápida y pruebas.
# IMPORTANTE: Este formato de doble modelo no resulto efectivo y no se usó en la versión final del TFG.
# se deja aquí como referencia.

import re
import sys
from pathlib import Path

import ollama

# Añadir la raíz del proyecto (carpeta 'tfg') al inicio de sys.path que funcionen los imports
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import (
    prompt_inicial_sin_ejemplos as first_prompt,
    validador_and_remake as second_prompt
)
from utils import select_model_ollama

model_1 = select_model_ollama()
model_2 = select_model_ollama()


chat = [
    {'role': 'user', 'content': first_prompt},
]

# Crear directorio logs si no existe
script_dir = Path(__file__).resolve().parent
logs_dir = script_dir / "logs"
logs_dir.mkdir(exist_ok=True)

# Primer modelo
response = ollama.chat(model=model_1, messages=chat)

# Limpiar <think>...</think> antes de añadirlo al historial
cleaned_content = re.sub(r'<think>.*?</think>', '', response['message']['content'], flags=re.DOTALL).strip()

chat.append({'role': 'assistant', 'content': cleaned_content})
print(cleaned_content)
with open(logs_dir / "logv2.txt", "a", encoding="utf-8") as f:
  f.write("\nMODEL_1 : " + str(model_1) + "\nPROMPT1:\n" + str(first_prompt) + "\nRESPONSE1:\n" + str(response['message']['content']) + "\n")
print("***********************************************")

# Segundo prompt
chat.append({'role': 'user', 'content': second_prompt})

# Segundo modelo
response = ollama.chat(model=model_2 , messages=chat)

# Limpiar de nuevo antes de usar
cleaned_content = re.sub(r'<think>.*?</think>', '', response['message']['content'], flags=re.DOTALL).strip()

chat.append({'role': 'assistant', 'content': cleaned_content})
print(cleaned_content)

with open(logs_dir / "logv2.txt", "a", encoding="utf-8") as f:
   f.writelines(["\nMODEL_2 : ", model_2 ,"\nPROMPT2:\n", second_prompt, "\nRESPONSE2:\n", cleaned_content])
