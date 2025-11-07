import json
import sys
from pathlib import Path

import ollama

# Configuración de path
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import prompt_inicial_sin_ejemplos
from utils import write_log_json, write_log_txt, select_model_ollama

client = ollama.Client()
model = select_model_ollama()

prompt = prompt_inicial_sin_ejemplos

response = client.generate(
    model=model,
    prompt=prompt,
    # stream=True,
    options={'temperature': 0.1},  # Make responses more deterministic
)

print(response['response'])
print("**********************\n", response)


# SI ACTIVAS EL STREAM=TRUE
# for part in response:
#     print(part['response'], end='', flush=True)

write_log_txt("log.txt", model, prompt, response)

write_log_json("log.json", model, prompt, response)