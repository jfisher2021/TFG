import ollama


client = ollama.Client()
# Listar todos los modelos disponibles
models = ollama.list()

# Ver todos los modelos
print("Modelos disponibles:\n")
for idx, model in enumerate(models['models']):
    print(f"{idx + 1}: {model['model']}")

# Solicitar al usuario seleccionar un modelo
model_index = int(input("Selecciona el número del modelo que quieres usar: ")) - 1

# Obtener el nombre del modelo seleccionado
selected_model = models['models'][model_index]['model']
print(f"Has seleccionado el modelo: {selected_model}")
model = selected_model

# Cargar archivos PDDL
# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/domain.pddl', 'r') as f:
#     domain = f.read()

# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/problem.pddl', 'r') as f:
#     problem = f.read()

# # Cargar CSV como texto plano
# with open('info.csv', 'r') as f:
#     csv_data = f.read()

prompt = f"""
You are a PDDL execution plan validator for a robot agent operating under battery constraints. Your task is to assess whether the provided plan is valid and correctly formatted based on the following rules.

✅ Allowed Actions (from the domain):
(start_welcome ?r)

Duration: 1

Preconditions: (initial_state ?r)

Effects: (can_start ?r)

(move ?r ?from ?to)

Duration: 15

Preconditions:

Robot must be at ?from

Battery ≥ 20

(can_start ?r)

Effects:

Battery decreases by 20

Robot is now at ?to

?to is marked as visited

(explain_painting ?r ?location)

Duration: 15

Preconditions:

Robot must be at ?location

Battery ≥ 10

(can_start ?r)

Effects:

Battery decreases by 10

explained_painting for that location is set

(recharge ?r ?location)

Duration: 5

Preconditions:

Robot must be at a location with (charger_at ?location)

Effects:

Battery is set to 100

🎯 Goal:
The goal of the plan is that the robot must:

Visit the following locations:
monalisa, elgrito, guernica, nocheestrellada

And explain the paintings at the following locations:
monalisa, elgrito, dibejo, guernica

📋 Validation Criteria:
Battery Constraints:

The robot starts with 100 battery units.

Each move consumes 20 battery units.

Each explain_painting consumes 10 battery units.

recharge restores battery to 100, but only at locations with a charger.

No action should be executed if it requires more battery than available.

Action Preconditions:

The robot must be at the correct location to move, recharge, or explain.

The robot must execute start_welcome to start the plan. It can only be executed once at the beginning.

The robot can only recharge at locations that have (charger_at ...).

Action Format:

Each action must follow the exact format:

<start_time>: (<action_name> <parameters>) [<duration>]
Timestamps must be strictly increasing.

Durations must match those defined in the domain.

🧠 Your Task:
Tell me if the plan is valid or invalid.

Tell me if the plan has the correct format.

If the plan is valid but the format is incorrect, return the same plan with corrected format.

If the plan is invalid (due to battery, logic, or syntax errors), return a corrected and valid plan that satisfies the goal and format.

PLAN TO EVALUATE 

"
0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home dibejo) [15.000]
16.002: (explain_painting tiago dibejo) [15.000]
31.002: (move tiago dibejo elgrito) [15.000]
46.002: (explain_painting tiago elgrito) [15.000]
61.002: (move tiago elgrito guernica) [15.000]
76.002: (explain_painting tiago guernica) [15.000]
91.003: (recharge tiago guernica) [5.000]
96.003: (move tiago guernica home) [15.000]
111.003: (recharge tiago home) [5.000]
116.004: (move tiago home monalisa) [15.000]
131.004: (explain_painting tiago monalisa) [15.000]
146.004: (move tiago monalisa nocheestrellada) [15.000]
"
"""


response = client.generate(
    model=model,
    prompt=prompt,
    # stream=True
    )

print(response['response'])
# for part in response:
#     print(part['response'], end='', flush=True)


# crear una archivo en python
f = open("log_validator.txt", "a")
f.writelines(["\nMODEL : ", model ,"\nPROMPT:\n", prompt, "\nRESPONSE:\n", response["response"]])
f.close()
  

import json

# Datos que deseas guardar
log_data = {
    "model": model,
    "prompt": prompt,
    "response": response["response"]
}

# Escribir en un archivo JSON
with open("log_validator.json", "a") as file:
    json.dump(log_data, file)
    file.write("\n")  # Nueva línea para separar cada registro