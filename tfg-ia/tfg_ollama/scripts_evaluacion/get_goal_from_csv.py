import ollama 
from ollama import chat
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from utils import select_model_ollama

client = ollama.Client()

# user_input = input('Chat with history: ')
if len(sys.argv) > 1:
    user_input = sys.argv[1:]
    user_input = ' '.join(user_input)
    print(f"User input: {user_input}")
else:
    print("No input provided. Exiting.")
    exit(1)

model = select_model_ollama()

# Cargar CSV como texto plano
with open('cuadros.csv', 'r', encoding='utf-8') as f:
    csv_data = f.read()

messages = [
    {
        'role': 'user',
        'content': f"""Here is the CSV file content: 
            {csv_data}
            This is de domain and problem PDDL files:
    dominio :
    "
    (define (domain library_domain)

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
    .
    .
    .
    .
    "

    problem:
    "
    (define (problem library_problem)
    (:domain library_domain)

    (:objects
   home monalisa nocheestrellada elgrito maestro_aprendiz guernica la_joven_de_la_perla las_meninas el_3_de_mayo_de_1808 
  el_jardin_de_las_delicias las_tres_gracias la_rendicion_de_breda el_nacimiento_de_venus
  la_creacion_de_adan la_ultima_cena la_libertad_guiando_al_pueblo el_hijo_del_hombre american_gothic 
  la_persistencia_de_la_memoria la_ronda_de_noche impresion_sol_naciente banistas_en_asnieres 
  saturno_devorando_a_su_hijo whistlers_mother la_gran_ola_de_kanagawa el_beso autorretrato_con_collar_de_espinas 
  el_carnaval_del_arlequin nighthawks retrato_de_adele_bloch_bauer_i campbells_soup_cans composition_viii - location
    tiago - robot
    )
    *
    *
    *
    "
    Your only task is to give me a goal in the form of a PDDL problem based on the content of the CSV file.
    In the CSV file, you will find the paintings.
    You have to act like a pddl planner and create a goal that includes visiting all the paintings and explaining them.
    
    You can use the following actions:
    - visit(robot, location)
    - explain_painting(robot, painting)

    You dont have to do the plan, just the goal.

    This is the process to follow:
    1. Select all the paintings from the CSV file.
    2. Search if the painting fits in what we want to do.
    3. Return a goal in the form of a PDDL problem.

    Whe you return the goal, you have to use the following format:
    GOAL:
    *
    *
    *

    Example:
    prmpt:
    "I want to visit all the spanish paintings an explain me all the barroco paintings of the museum"
    Goal:
    "(visit tiago ....) (....) (explain_painting tiago las_meninas) ..."
    """,
    }
]

response = client.generate(
    model=model,
    prompt=messages[-1]["content"] + user_input,
    # stream=True
    )

print(response['response'])
