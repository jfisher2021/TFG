from ollama import Client
import sys
import os

client = Client()
with open('/home/jfisher/tfg/cuadros.csv', 'r') as f:
    csv_data = f.read()
response = client.create(
  model='my-assistant',
  from_='command-a:latest',
    
  system=f"""

    Here is the CSV file content: 
    {csv_data}
    This is a CSV file with information about paintings, including their names, authors, and locations.
    The plan is going to be used in a museum context, where a robot named Tiago will be moving around different locations in the museum to explain paintings to visitors.

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
        
    )
    )
    "


    Im going to give you 2 examples with their output format, so you can understand how to do it.


    EXAMPLE 1:
    Goal:
        (:goal
        (and

        (visited tiago monalisa)
        (visited tiago elgrito)
        (visited tiago guernica)
        (visited tiago nocheestrellada)
        (visited tiago dibejo)

        (explained_painting monalisa)
        (explained_painting dibejo)
        (explained_painting guernica)
        
    
        )
        )
    PLAN:
    0.000: (start_welcome tiago)  [1.000]
    1.001: (move tiago home dibejo)  [15.000]
    16.002: (explain_painting tiago dibejo)  [15.000]
    31.002: (move tiago dibejo guernica)  [15.000]
    46.002: (explain_painting tiago guernica)  [15.000]
    61.002: (move tiago guernica home)  [15.000]
    76.002: (recharge tiago home)  [5.000]
    81.003: (move tiago home elgrito)  [15.000]
    96.004: (move tiago elgrito monalisa)  [15.000]
    111.004: (explain_painting tiago monalisa)  [15.000]
    126.004: (move tiago monalisa nocheestrellada)  [15.000]


    EXAMPLE 2:
    Goal:
    :goal
    (and

        (visited tiago monalisa)
        (visited tiago guernica)
        (visited tiago dibejo)

        (explained_painting monalisa)
        (explained_painting dibejo)
        (explained_painting guernica)
        (explained_painting nocheestrellada)
        (explained_painting elgrito)
        
  
        )
    )
        
    PLAN:
    0.000: (start_welcome tiago)  [1.000]
    1.001: (move tiago home dibejo)  [15.000]
    16.002: (explain_painting tiago dibejo)  [15.000]
    31.002: (move tiago dibejo elgrito)  [15.000]
    46.002: (explain_painting tiago elgrito)  [15.000]
    61.002: (move tiago elgrito home)  [15.000]
    76.002: (recharge tiago home)  [5.000]
    81.003: (move tiago home guernica)  [15.000]
    96.003: (explain_painting tiago guernica)  [15.000]
    111.003: (move tiago guernica monalisa)  [15.000]
    126.003: (explain_painting tiago monalisa)  [15.000]
    141.003: (move tiago monalisa nocheestrellada)  [15.000]
    156.003: (explain_painting tiago nocheestrellada)  [15.000]




    I HAVEN'T PUT THE GOAL IN THE PROBLEM, THE GOAL IM GOING TO PASS IT TO YOU LATER AS AN INPUT.


    The most important thing in the plan is that you consider the battery.
    Starting with 100, moving consumes 20, and explaining consumes 10. Therefore, you must recharge the robot before the battery runs out. If you have 20 battery, the action must be MOVE TO HOME to recharge.

    REMEMBER, THE OUTPUT MUST ALWAYS HAVE THIS FORMAT.

    0.000: (start_welcome tiago) [1.000]
    1.001: (move tiago home dibeºjo) [15.000]
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

    """,
  stream=False,
)
print(response.status)