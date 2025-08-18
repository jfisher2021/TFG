from langchain_core.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
import ollama
import json

# Listar todos los modelos disponibles usando ollama directamente
client = ollama.Client()
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

# Crear el modelo de LangChain con Ollama
model = ChatOllama(
    model=selected_model,
    temperature=0.0,
)

# Prompt PDDL
pddl_prompt = """
You are a planner for PDDL and I only want you to give me the execution plan. You need to take into account the following:

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
    (visited tiago la_libertad_guiando_al_pueblo)
    (explained_painting guernica)
    (explained_painting el_3_de_mayo_de_1808)
    (explained_painting la_rendicion_de_breda)
    (explained_painting el_jardin_de_las_delicias)
    (explained_painting la_ultima_cena)
    (explained_painting la_persistencia_de_la_memoria)
    (explained_painting el_nacimiento_de_venus)
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


Give me de step by step explanation of when you should recharge de robot, when you are planning and explaining a painting,
tell me why you are explaining the drawing and if it was in the goal or not. 

But at the end do:
"
PLAN: 
(AND GIVE THE PLAN WITH CORRECT FORMAT)
"
"""

# Generar respuesta usando LangChain
print("\nGenerando plan PDDL...")
print("=" * 50)

# Usar streaming para mostrar la respuesta en tiempo real
for chunk in model.stream(pddl_prompt):
    print(chunk.content, end='', flush=True)

# Para obtener la respuesta completa para guardar en archivos
response = model.invoke(pddl_prompt)
full_response = response.content

print("\n\n" + "=" * 50)
print("Guardando logs...")

# Guardar en archivo de texto
with open("log.txt", "a", encoding='utf-8') as f:
    f.writelines(["\nMODEL : ", selected_model, "\nPROMPT:\n", pddl_prompt, "\nRESPONSE:\n", full_response, "\n" + "="*50 + "\n"])

# Guardar en archivo JSON
log_data = {
    "model": selected_model,
    "prompt": pddl_prompt,
    "response": full_response
}

with open("log.json", "a", encoding='utf-8') as file:
    json.dump(log_data, file, ensure_ascii=False, indent=2)
    file.write("\n")

print("Logs guardados en log.txt y log.json")