# Script para validar un plan específico 

import json
import sys
from pathlib import Path

import ollama

# Añadir la raíz del proyecto (carpeta 'tfg') al inicio de sys.path que funcionen los imports
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import validator_simple
from utils import select_model_ollama, write_log_txt, write_log_json

client = ollama.Client()
model = select_model_ollama()

prompt = validator_simple

response = client.generate(
    model=model,
    prompt=prompt,
    # stream=True
    )

print(response['response'])

# SI ACTIVAS EL STREAM=TRUE
# for part in response:
#     print(part['response'], end='', flush=True)

# crear una archivo en python
write_log_txt("log_validator.txt", model, prompt, response)
write_log_json("log_validator.json",model, prompt, response)