import json
import sys
from pathlib import Path

import ollama

# Añadir la raíz del proyecto (carpeta 'tfg') al inicio de sys.path que funcionen los imports
ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import prompt_con_3_ejemplos_input_goal
from utils import select_model_ollama, write_log_txt, write_log_json

client = ollama.Client()

def main():
    if len(sys.argv) > 1:
        goal = " ".join(sys.argv[1:])
    else:
        goal = """    
        (explain_painting tiago monalisa)
        (visited tiago guernica)
        (explained_painting elgrito)
        """
    print("Goal to achieve: ", goal)
    
    model = select_model_ollama()
    prompt = prompt_con_3_ejemplos_input_goal.format(GOAL=goal)

    response = client.generate(
        model=model,
        prompt=prompt,
        # stream=True
        options={
            'temperature': 0.1 # Make responses more deterministic MAX 1 and MIN 0
        }
    )

    print(response['response'])
    
    write_log_txt("log.txt", model, prompt, response)
    write_log_json("log.json", model, prompt, response)

if __name__ == "__main__":
    main()