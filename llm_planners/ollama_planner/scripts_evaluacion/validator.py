# Script para validar un plan específico 
# IMPORTANTE: este script de ollama es un validador para validar un plan generado por otro LLM
# No se usa en la versión final del TFG, se deja aquí como referencia, y puede ser útil para futuros trabajos.

import json
import sys
from pathlib import Path

import ollama

# Añadir la raíz del proyecto (carpeta 'tfg') al inicio de sys.path que funcionen los imports
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import validator_simple
from utils import select_model_ollama

client = ollama.Client()
model = select_model_ollama()
plan = """
0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home maestro_aprendiz) [15.000]
16.002: (explain_painting tiago maestro_aprendiz) [15.000]
31.002: (move tiago maestro_aprendiz elgrito) [15.000]
46.002: (explain_painting tiago elgrito) [15.000]
61.002: (move tiago elgrito guernica) [15.000]
76.002: (explain_painting tiago guernica) [15.000]
91.003: (recharge tiago guernica) [5.000]
96.003: (move tiago guernica home) [15.000]
111.003: (recharge tiago home) [5.000]
116.004: (move tiago home monalisa) [15.000]
131.004: (explain_painting tiago monalisa) [15.000]
146.004: (move tiago monalisa nocheestrellada) [15.000]
"""

prompt = validator_simple.format(PLAN=plan)

response = client.generate(
    model=model,
    prompt=prompt,
    # stream=True
    )

print(response['response'])

# SI ACTIVAS EL STREAM=TRUE
# for part in response:
#     print(part['response'], end='', flush=True)

