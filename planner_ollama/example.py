import ollama


client = ollama.Client()
model = "llama3.2:1b"

# Cargar archivos PDDL
# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/domain.pddl', 'r') as f:
#     domain = f.read()

# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/problem.pddl', 'r') as f:
#     problem = f.read()

# # Cargar CSV como texto plano
# with open('info.csv', 'r') as f:
#     csv_data = f.read()

prompt = f"""
vas a hacer de planificador  para pddl y quiero que me des un plan, SOLO QUIERO QUE ME DES EL PLAN. te voy a pasar el dominio y el problem. 

dominio :
"
(define (domain library_domain)

(:requirements :strips :fluents :durative-actions :typing)

(:types 
  robot
  location 
  painting
  person
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

    (over all  (robot_at ?r   ?p))
    (at start (can_start ?r))
  )
  :effect (and 
    (at end (explained_painting ?p)) 
    (at start (decrease (battery ?r) 10))
  )
)


(:durative-action search_painting
  :parameters (?r - robot ?b - painting ?l - location)
  :duration (= ?duration 20)
  :condition (and 
    (over all (robot_at ?r ?l))
    (over all  (painting_at ?b ?l))
    (at start (can_start ?r))

  )
  :effect (and 
    (at end (painting_found ?b))
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

)



; ACCIONES: 

; - Saludar a un visitante (EMPEZAR DANDO LA BIENBENIDA)
; - Mover el robot a una ubicación
; - Explicar una pintura
; - Recargar la batería

"

porblem: 

"
(define (problem library_problem)
(:domain library_domain)

(:objects
  home monalisa_ws nocheestrellada_ws elgrito_ws dibejo_ws guernica_ws - location

  tiago - robot
  pers_1 - person
)

(:init
  ; Robot initialization
  (robot_at tiago home)
  (initial_state tiago)

  ; Connections
  ; (finished_explain tiago)

  ; Object locations
  (= (battery tiago) 100)
  (person_at pers_1 guernica_ws)
  (not(person_attended pers_1))
  (charger_at home)
)

(:goal
  (and
    ; (robot_at tiago guernica_ws)

    ; (person_attended pers_1)
    (explained_painting monalisa_ws)
    (visited tiago monalisa_ws)
    (visited tiago elgrito_ws)
    (visited tiago guernica_ws)
    (visited tiago nocheestrellada_ws)
    (visited tiago dibejo_ws)
    ; (explained_painting nocheestrellada_ws)
    (explained_painting elgrito_ws)
    ; (explained_painting dibejo_ws)
    (explained_painting guernica_ws)
    ; (finished_explain tiago)
  )
)

)
"

REPITO: SOLO QUIERO QUE ME DES EL PLAN 

Un ejmeplo de plan es el siguiente:
"
0.000: (start_welcome tiago)  [1.000]
1.001: (move tiago home dibejo_ws)  [15.000]
16.002: (move tiago dibejo_ws elgrito_ws)  [15.000]
31.002: (explain_painting tiago elgrito_ws)  [15.000]
46.002: (move tiago elgrito_ws guernica_ws)  [15.000]
.
.
.
"
NO esta acabado el plan, pero es un ejemplo de como es el plan.

QUIERO QUE SOLO ME CONTESTES CON EL PLAN, NADA MAS.
RESPUESTA ="
"

"""

response = client.generate(
    model=model,
    prompt=prompt,
    stream=True)

for part in response:
    print(part['response'], end='', flush=True)
# The above code is a simple example of how to use the Ollama client to generate text.
# It initializes the client, sets the model and prompt, and then generates a response.
# The response is printed to the console.
# Note: The model name and prompt can be changed to suit your needs.
# The temperature parameter controls the randomness of the output.
# A higher temperature (e.g., 1.0) will produce more random outputs, while a lower temperature (e.g., 0.2) will produce more focused and deterministic outputs.


################################ PLAN REAL ######################################

# 0.000: (start_welcome tiago)  [1.000]
# 1.001: (move tiago home dibejo_ws)  [15.000]
# 16.002: (move tiago dibejo_ws elgrito_ws)  [15.000]
# 31.002: (explain_painting tiago elgrito_ws)  [15.000]
# 46.002: (move tiago elgrito_ws guernica_ws)  [15.000]
# 61.002: (explain_painting tiago guernica_ws)  [15.000]
# 76.002: (move tiago guernica_ws home)  [15.000]
# 91.002: (recharge tiago home)  [5.000]
# 96.003: (move tiago home monalisa_ws)  [15.000]
# 111.003: (explain_painting tiago monalisa_ws)  [15.000]
# 126.003: (move tiago monalisa_ws nocheestrellada_ws)  [15.000]
