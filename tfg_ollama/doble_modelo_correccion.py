import ollama
import re
import yaml


general_prompt= f"""
You are a planner for PDDL and I only want you to give me the execution plan. You need to take into account the following:

I will provide you with the domain and the problem. The only output I expect from you is the execution plan, as a set of actions, with their start times and durations. The structure of the plan should be the following:

0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home dibejo) [15.000]
16.002: (move tiago dibejo elgrito) [15.000]
31.002: (explain_painting tiago elgrito) [15.000]
46.002: (move tiago elgrito guernica) [15.000]
...
Now I will provide the domain and the problem. ONLY RESPOND WITH THE PLAN, NOTHING ELSE.

dominio :
"
(define (domain library_domain)

(:requirements :strips :fluents :durative-actions :typing)

(:types 
  robot
  location 
  painting
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (painting_at ?b - painting ?l - location)
  (painting_found ?b - painting)
  (explained_painting ?p - location)
  (can_start ?r - robot)
  (initial_state ?r - robot)
  (visited ?r - robot ?l - location)
  (charger_at ?wp - location)
)

(:functions
  (battery ?r - robot)
)

(:durative-action start_welcome
  :parameters (?r - robot)
  :duration (= ?duration 1)
  :condition (and 
    (at start (initial_state ?r))
  )
  :effect (and 
    (at end (can_start ?r))
    (at start (initial_state ?r))
  )
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 15)
  :condition (and 
    (at start (>= (battery ?r) 20))
    (at start (can_start ?r))
    (at start (robot_at ?r ?from))
  )
  :effect (and 
    (at start (not(robot_at ?r ?from)))
    (at start (decrease (battery ?r) 20))
    (at end (robot_at ?r ?to))
    (at end (visited ?r ?to))
  )
)

(:durative-action explain_painting
  :parameters (?r - robot ?p - location)
  :duration (= ?duration 15)
  :condition (and 
    (at start (>= (battery ?r) 10))
    (over all (robot_at ?r   ?p))
    (at start (can_start ?r))
  )
  :effect (and 
    (at end (explained_painting ?p)) 
    (at start (decrease (battery ?r) 10))
  )
)

(:durative-action recharge
  :parameters (?r - robot ?wp - location)
  :duration (= ?duration 5)
  :condition (and 
    (at start (<= (battery ?r) 100))
    (over all (robot_at ?r ?wp))
    (over all (charger_at ?wp))
  )
  :effect (and (at end (assign (battery ?r) 100)))
)
"

problem:
"
(define (problem library_problem)
(:domain library_domain)

(:objects
 home monalisa nocheestrellada elgrito dibejo guernica la_joven_de_la_perla las_meninas el_3_de_mayo_de_1808 
  el_jardin_de_las_delicias las_tres_gracias la_rendicion_de_breda el_nacimiento_de_venus
  la_creacion_de_adan la_ultima_cena la_libertad_guiando_al_pueblo el_hijo_del_hombre american_gothic 
  la_persistencia_de_la_memoria la_ronda_de_noche impresion_sol_naciente banistas_en_asnieres 
  saturno_devorando_a_su_hijo whistlers_mother la_gran_ola_de_kanagawa el_beso autorretrato_con_collar_de_espinas 
  el_carnaval_del_arlequin nighthawks retrato_de_adele_bloch_bauer_i campbells_soup_cans composition_viii - location
  tiago - robot
)

(:init
  (robot_at tiago home)
  (initial_state tiago)
  (= (battery tiago) 100)
  (charger_at home)
)

(:goal
  (and
    (visited tiago monalisa)
    (visited tiago elgrito)
    (visited tiago guernica)
    (visited tiago nocheestrellada)
    (explained_painting monalisa)
    (explained_painting elgrito)
    (explained_painting dibejo)
    (explained_painting guernica)
  )
)
"


The most important thing in the plan is that you consider the battery.
Starting with 100, moving consumes 20, and explaining consumes 10. Therefore, you must recharge the robot before the battery runs out. If you have 20 battery, the action must be MOVE TO HOME to recharge.

REMEMBER, THE OUTPUT MUST ALWAYS HAVE THIS FORMAT.

0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home dibejo) [15.000]
16.002: (move tiago dibejo elgrito) [15.000]
31.002: (explain_painting tiago elgrito) [15.000]
46.002: (move tiago elgrito guernica) [15.000]
"""
first_prompt = f"""You are a planner for PDDL and I only want you to give me the execution plan. You need to take into account the following:

1.The robot has a battery that is consumed with each movement and action.
2.The battery must be recharged when necessary, but you should minimize the number of recharges and movements to make the plan as efficient as possible.
3.Maximize efficiency regarding battery usage, performing recharges only when absolutely necessary and only at locations that have chargers.
4.Do not make any comments or additional explanations. I only want the plan.
5.Do not invent actions or make unnecessary actions.

I will provide you with the domain and the problem. The only output I expect from you is the execution plan, as a set of actions, with their start times and durations. The structure of the plan should be the following:

0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home dibejo) [15.000]
16.002: (move tiago dibejo elgrito) [15.000]
31.002: (explain_painting tiago elgrito) [15.000]
46.002: (move tiago elgrito guernica) [15.000]
...
ONLY RESPOND WITH THE PLAN, NOTHING ELSE.

The most important thing in the plan is that you consider the battery.
Starting with 100, moving consumes 20, and explaining consumes 10. Therefore, you must recharge the robot before the battery runs out. If you have 20 battery, the action must be MOVE TO HOME to recharge.

REMEMBER, THE OUTPUT MUST ALWAYS HAVE THIS FORMAT.

0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home dibejo) [15.000]
16.002: (move tiago dibejo elgrito) [15.000]
31.002: (explain_painting tiago elgrito) [15.000]
46.002: (move tiago elgrito guernica) [15.000]


Give me de step by step explanation of when you should recharge de robot, when you are planning and explaining a painting,
tell me why you are explaining the drawing and if it was in the goal or not. 

But at the end do:

PLAN: 
(AND GIVE THE PLAN WITH CORRECT FORMAT)

"""

second_prompt= f"""You are a PDDL execution plan validator for a robot agent operating under battery constraints. Your task is to assess whether the provided plan is valid and correctly formatted based on the following rules:
Action Syntax and Format:
Each action must follow this structure exactly:

<start_time>: (<action_name> <parameters>) [<duration>]

Times and durations must be numeric and strictly increasing.

Durations must match those defined in the domain (e.g., move = 20, explain_painting = 15, recharge = 5, start_welcome = 1).

THE FIRST THING THAT I WANT YOU TO DO IS COPY THE PLAN THAT I GIVE YOU AND PUT IT WHITH THE CORRECT FORMAT. IT DOESNT MATTER IF ITS INVALID

ONCE YOU HAVE THE CORRECT FORMAT CHEK IF IS VALID
The robot starts with 100 battery units.
Each move action consumes 20 battery units.
Each explain_painting action consumes 10 battery units.
recharge resets the battery to 100, but can only occur at locations with a charger.
The robot must never attempt an action that requires more battery than it has available at that time.

Location and Preconditions:

The robot must be at the correct location before performing actions such as explain_painting or recharge.
The robot can only move if it has enough battery and has executed the start_welcome action.



Your Task:
The first thing you have to do is tell me de goal of the plan.
Once you put the goal, check if the goal is completed or not.
Tell me if the plan is valid or invalid.
Tell me if the plan has the correct format.

In all of tha cases you have to put which exact line or action is wrong, and the change it to a bettr one.

If the plan is valid but the format is incorrect, return the same plan with the corrected format.
If the plan is invalid (due to battery misuse, wrong preconditions, or invalid syntax), correct it and return a valid, properly formatted plan.

You also have to modify the plan if some of the actions are unnecesary or they explain or visit drawings that are not in the plan.
You have to see if all the goals are covered.
Only respond with the validation results and, if applicable, the corrected plan.


The final thing after you explain everything you have tu say:

FINAL PLAN:
(PUT PLAN)
"""


# Simulamos un modelo y prompts (ajusta según tu flujo real)
model_2 = 'llama4:latest'


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
model_1 = selected_model


chat = [
    {'role': 'system', 'content': general_prompt},
    {'role': 'user', 'content': first_prompt},
]

# Primer modelo
response = ollama.chat(model=model_1, messages=chat)
f = open("logv2.txt", "a")

# Limpiar <think>...</think> antes de añadirlo al historial
cleaned_content = re.sub(r'<think>.*?</think>', '', response['message']['content'], flags=re.DOTALL).strip()

chat.append({'role': 'assistant', 'content': cleaned_content})
print(cleaned_content)
f.writelines(["\nMODEL_1 : ", model_1 ,"\nPROMPT1:\n", first_prompt, "\nRESPONSE1:\n", response['message']['content']])
print("***********************************************")

# Segundo prompt
chat.append({'role': 'user', 'content': second_prompt})

# Segundo modelo
response = ollama.chat(model='command-a:latest', messages=chat)

# Limpiar de nuevo antes de usar
cleaned_content = re.sub(r'<think>.*?</think>', '', response['message']['content'], flags=re.DOTALL).strip()

chat.append({'role': 'assistant', 'content': cleaned_content})
print(cleaned_content)


# crear una archivo en python
f.writelines(["\nMODEL_2 : ", model_2 ,"\nPROMPT2:\n", second_prompt, "\nRESPONSE2:\n", cleaned_content])
f.close()

# import json

# # Datos que deseas guardar
# log_data = {
#     "model": model,
#     "prompt": prompt,
#     "response": response["response"]
# }

# # Escribir en un archivo JSON
# with open("logv2.json", "a") as file:
#     json.dump(log_data, file)
#     file.write("\n")  # Nueva línea para separar cada registro

# Guardar en archivo YAML
log_data = {
    "model_sequence": [model_1, model_2],
    "conversation": chat
}

with open("logv2.yaml", "a") as file:
    yaml.dump(log_data, file, allow_unicode=True, sort_keys=False)