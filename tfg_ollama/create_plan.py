import ollama
import sys
import os
import json

# Añadir la raíz del proyecto (carpeta 'tfg') al inicio de sys.path para que `from prompts import ...` funcione
# Esto permite ejecutar el script desde cualquier directorio y que Python encuentre `tfg/prompts.py`.
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from prompts import prompt_con_3_ejemplos

client = ollama.Client()
# Listar todos los modelos disponibles
models = ollama.list()
def select_model():
    # Ver todos los modelos
    print("Modelos disponibles:\n")
    for idx, model in enumerate(models['models']):
        print(f"{idx + 1}: {model['model']}")

    # Solicitar al usuario seleccionar un modelo
    # model_index = int(input("Selecciona el número del modelo que quieres usar: ")) - 1
    model_index = 1
    # Obtener el nombre del modelo seleccionado
    selected_model = models['models'][model_index]['model']
    print(f"Has seleccionado el modelo: {selected_model}")
    model = selected_model
    return model
# Cargar archivos PDDL
# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/domain.pddl', 'r') as f:
#     domain = f.read()

# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/problem.pddl', 'r') as f:
#     problem = f.read()

# # Cargar CSV como texto plano
# with open('info.csv', 'r') as f:
#     csv_data = f.read()


def main():
    if len(sys.argv) > 1:
        goal = " ".join(sys.argv[1:])
    else:
        goal = """    (explain_painting tiago monalisa)
        (visited tiago guernica)
        (explained_painting elgrito)"""
    print("Goal to achieve: ", goal)
    
    model = select_model()
    prompt = prompt_con_3_ejemplos.format(GOAL=goal)

    response = client.generate(
        model=model,
        prompt=prompt,
        # stream=True
        options={
            'temperature': 0.3 # Make responses more deterministic MAX 1 and MIN 0
        }
        )

    print(response['response'])



    # crear una archivo en python
    f = open("log.txt", "a")
    f.writelines(["\nMODEL : ", model ,"\nPROMPT:\n", prompt, "\nRESPONSE:\n", response["response"]])
    f.close()


    # Datos que deseas guardar
    log_data = {
        "model": model,
        "prompt": prompt,
        "response": response["response"]
    }

    # Escribir en un archivo JSON
    with open("log.json", "a") as file:
        json.dump(log_data, file)
        file.write("\n")  # Nueva línea para separar cada registro

if __name__ == "__main__":
    main()