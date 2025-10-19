
prompt_inicial_sin_ejemplos = f"""
You are a planner for PDDL and I only want you to give me the execution plan. You need to take into account the following:

1.The robot has a battery that is consumed with each movement and action.
2.The battery must be recharged when necessary, but you should minimize the number of recharges and movements to make the plan as efficient as possible.
3.Maximize efficiency regarding battery usage, performing recharges only when absolutely necessary and only at locations that have chargers.
4.Do not make any comments or additional explanations. I only want the plan.
5.Do not invent actions or make unnecessary actions.

I will provide you with the domain and the problem. The only output I expect from you is the execution plan, as a set of actions, with their start times and durations. The structure of the plan should be the following:

0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home maestro_aprendiz) [15.000]
16.002: (move tiago maestro_aprendiz elgrito) [15.000]
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
  home monalisa nocheestrellada elgrito maestro_aprendiz guernica la_joven_de_la_perla las_meninas el_3_de_mayo_de_1808 
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
    (explained_painting maestro_aprendiz)
    (explained_painting guernica)
    (explained_painting la_joven_de_la_perla)
    (explained_painting las_meninas)
    (explained_painting el_3_de_mayo_de_1808)
    (explained_painting el_jardin_de_las_delicias)
  
  )
)
"

The most important thing in the plan is that you consider the battery.
Starting with 100, moving consumes 20, and explaining consumes 10. Therefore, you must recharge the robot before the battery runs out. If you have 20 battery, the action must be MOVE TO HOME to recharge.

REMEMBER, THE OUTPUT MUST ALWAYS HAVE THIS FORMAT.

0.000: (start_welcome tiago) [1.000]
1.001: (move tiago home maestro_aprendiz) [15.000]
16.002: (move tiago maestro_aprendiz elgrito) [15.000]
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

prompt_con_3_ejemplos = f"""

    You are a planner for PDDL and I only want you to give me the execution plan. You need to take into account the following:

    1.The robot has a battery that is consumed with each movement and action.
    2.The battery must be recharged when necessary, but you should minimize the number of recharges and movements to make the plan as efficient as possible.
    3.Maximize efficiency regarding battery usage, performing recharges only when absolutely necessary and only at locations that have chargers.
    4.Do not make any comments or additional explanations. I only want the plan.
    5.Do not invent actions or make unnecessary actions.

    I will provide you with the domain and the problem. The only output I expect from you is the execution plan, as a set of actions, with their start times and durations. The structure of the plan should be the following:

    0.000: (start_welcome tiago) [1.000]
    1.001: (move tiago home maestro_aprendiz) [15.000]
    16.002: (move tiago maestro_aprendiz elgrito) [15.000]
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
    home monalisa nocheestrellada elgrito maestro_aprendiz guernica la_joven_de_la_perla las_meninas el_3_de_mayo_de_1808 
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


    GOOD EXAMPLE 1:
    "
    Goal:
        (:goal
        (and

        (visited tiago monalisa)
        (visited tiago elgrito)
        (visited tiago guernica)
        (visited tiago nocheestrellada)
        (visited tiago maestro_aprendiz)

        (explained_painting monalisa)
        (explained_painting maestro_aprendiz)
        (explained_painting guernica)
        
    
        )
        )
    PLAN:
    0.000: (start_welcome tiago)  [1.000]
    1.001: (move tiago home maestro_aprendiz)  [15.000]
    16.002: (explain_painting tiago maestro_aprendiz)  [15.000]
    31.002: (move tiago maestro_aprendiz guernica)  [15.000]
    46.002: (explain_painting tiago guernica)  [15.000]
    61.002: (move tiago guernica home)  [15.000]
    76.002: (recharge tiago home)  [5.000]
    81.003: (move tiago home elgrito)  [15.000]
    96.004: (move tiago elgrito monalisa)  [15.000]
    111.004: (explain_painting tiago monalisa)  [15.000]
    126.004: (move tiago monalisa nocheestrellada)  [15.000]
    "


    GOOD EXAMPLE 2:
    "
    Goal:
    :goal
    (and

        (visited tiago monalisa)
        (visited tiago guernica)
        (visited tiago maestro_aprendiz)

        (explained_painting monalisa)
        (explained_painting maestro_aprendiz)
        (explained_painting guernica)
        (explained_painting nocheestrellada)
        (explained_painting elgrito)
        
  
        )
    )
        
    PLAN:
    0.000: (start_welcome tiago)  [1.000]
    1.001: (move tiago home maestro_aprendiz)  [15.000]
    16.002: (explain_painting tiago maestro_aprendiz)  [15.000]
    31.002: (move tiago maestro_aprendiz elgrito)  [15.000]
    46.002: (explain_painting tiago elgrito)  [15.000]
    61.002: (move tiago elgrito home)  [15.000]
    76.002: (recharge tiago home)  [5.000]
    81.003: (move tiago home guernica)  [15.000]
    96.003: (explain_painting tiago guernica)  [15.000]
    111.003: (move tiago guernica monalisa)  [15.000]
    126.003: (explain_painting tiago monalisa)  [15.000]
    141.003: (move tiago monalisa nocheestrellada)  [15.000]
    156.003: (explain_painting tiago nocheestrellada)  [15.000]
    "

    BAD EXAMPLE 1:
    "
    Goal:
    (visited tiago monalisa)
    (visited tiago elgrito)
    (visited tiago guernica)
    (visited tiago nocheestrellada)
    (explained_painting monalisa)
    (explained_painting elgrito)
    (explained_painting maestro_aprendiz)
    (explained_painting guernica)

    BAD PLAN:
    0.000: (start_welcome tiago) [1.000]
    1.001: (move tiago home maestro_aprendiz) [15.000]
    16.002: (explain_painting tiago maestro_aprendiz) [15.000]
    31.002: (move tiago maestro_aprendiz elgrito) [15.000]
    46.002: (explain_painting tiago elgrito) [15.000]
    61.002: (move tiago elgrito guernica) [15.000]
    76.002: (explain_painting tiago guernica) [15.000]
    91.002: (move tiago guernica home) [15.000]
    106.003: (recharge tiago home) [5.000]
    111.003: (move tiago home monalisa) [15.000]
    126.003: (explain_painting tiago monalisa) [15.000]

    Explanation:
    The plan is incorrect because the robot does not have enough battery to home in this action:
    76.002: (explain_painting tiago guernica) [15.000]
    91.002: (move tiago guernica home) [15.000]
    The robot moved with 10 battery, bur it needs 20 battery to move.
    "


    THE GOAL IS 
    \"\"\"{{GOAL}}\"\"\"

    The most important thing in the plan is that you consider the battery.
    Starting with 100, moving consumes 20, and explaining consumes 10. Therefore, you must recharge the robot before the battery runs out. If you have 20 battery, the action must be MOVE TO HOME to recharge.

    REMEMBER, THE OUTPUT MUST ALWAYS HAVE THIS FORMAT.

    0.000: (start_welcome tiago) [1.000]
    1.001: (move tiago home maestro_aprendiz) [15.000]
    16.002: (move tiago maestro_aprendiz elgrito) [15.000]
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

prompt_para_fichero_sin_goal = f"""
    You are a planner for PDDL and I only want you to give me the execution plan. You need to take into account the following:

    1.The robot has a battery that is consumed with each movement and action.
    2.The battery must be recharged when necessary, but you should minimize the number of recharges and movements to make the plan as efficient as possible.
    3.Maximize efficiency regarding battery usage, performing recharges only when absolutely necessary and only at locations that have chargers.
    4.Do not make any comments or additional explanations. I only want the plan.
    5.Do not invent actions or make unnecessary actions.

    I will provide you with the domain and the problem. The only output I expect from you is the execution plan, as a set of actions, with their start times and durations. The structure of the plan should be the following:

    0.000: (start_welcome tiago) [1.000]
    1.001: (move tiago home maestro_aprendiz) [15.000]
    16.002: (move tiago maestro_aprendiz elgrito) [15.000]
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
    home monalisa nocheestrellada elgrito maestro_aprendiz guernica la_joven_de_la_perla las_meninas el_3_de_mayo_de_1808 
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
     The most important thing in the plan is that you consider the battery.
    Starting with 100, moving consumes 20, and explaining consumes 10. Therefore, you must recharge the robot before the battery runs out. If you have 20 battery, the action must be MOVE TO HOME to recharge.

    REMEMBER, THE OUTPUT MUST ALWAYS HAVE THIS FORMAT.

    0.000: (start_welcome tiago) [1.000]
    1.001: (move tiago home maestro_aprendiz) [15.000]
    16.002: (move tiago maestro_aprendiz elgrito) [15.000]
    31.002: (explain_painting tiago elgrito) [15.000]
    46.002: (move tiago elgrito guernica) [15.000]


    Give me de step by step explanation of when you should recharge de robot, when you are planning and explaining a painting,
    tell me why you are explaining the drawing and if it was in the goal or not. 

    But at the end do:
    "
    PLAN: 
    (AND GIVE THE PLAN WITH CORRECT FORMAT)
    "


    the goal is to visit all the spanish drawings and explain all the barroco drawings not only the spanish ones.
    """


validate_plan_prompt = f"""
You are a PDDL plan validator. Your job is to verify that the given execution plan is valid and logically consistent with the domain, the problem, and the constraints described below.

### You must validate the following:

1. **Battery Logic**:
   - The robot starts with 100 battery.
   - Each `(move ...)` action consumes 20 battery units.
   - Each `(explain_painting ...)` action consumes 10 battery units.
   - Each `(recharge ...)` action restores battery to 100.
   - Battery level must **never drop below 0** at any point in the plan.
   - You must simulate the battery consumption **step by step**, tracking battery level after each action.

2. **Action Integrity**:
   - All actions must be strictly from the domain. Do not allow made-up actions.
   - Action names, parameters, and structure must exactly match the domain definitions.
   - Locations and painting names must match exactly those defined in the problem.
   - Do not allow typos, non-existent paintings, or made-up locations.

3. **Goal Achievement**:
   - All goals in the `(:goal ...)` section of the problem must be fulfilled.
   - No extra goals or explanations should appear.
   - Each `explained_painting` must refer to a location actually defined in the goal.
   - Each `visited` location must match exactly what is required in the goal.

4. **Temporal Consistency**:
   - Start times must be non-decreasing.
   - Durations must match exactly those defined in the domain:
     - `(start_welcome ...)` duration = 1
     - `(move ...)` duration = 15
     - `(explain_painting ...)` duration = 15
     - `(recharge ...)` duration = 5
   - Time stamps must reflect continuous execution or valid gaps, with no overlaps.

5. **Plan Format**:
   - Plan must follow this exact format:
     ```
     0.000: (start_welcome tiago) [1.000]
     1.001: (move tiago home maestro_aprendiz) [15.000]
     ...
     ```

6. **Charger Locations**:
   - Recharge actions must only be done at locations that have a charger (as per `charger_at`).
   - Recharging must happen **before** the battery would otherwise go negative.

### Your Output

Please give a detailed step-by-step validation:
- Track and report the battery after each action.
- Indicate whether the action is valid and why.
- Highlight any action that breaks domain constraints.
- Indicate whether each goal is achieved.
- Explicitly state whether the plan is valid or not.

If the plan is invalid, **clearly explain why**.


Here is the domain, the problem, and the execution plan:

DOMAIN:
\"\"\"
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


\"\"\"

PROBLEM:
\"\"\"

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

(:init
  ; Robot initialization
  (robot_at tiago home)
  (initial_state tiago)

  ; Object locations
  (= (battery tiago) 100)
  (charger_at home)
)


)

Finalmente, estas diseñado para meter los resultados en un csv, por lo tanto tu salida esta estructurada en formato csv.

Por ejemplo algo asi :
Modelo,Intento,Goal,Plan_raw,Formato_valido,Cumple_goal,Plan_Valido,Errores,Comentarios
gemini-2.5-flash,1,"Visitar 4 cuadros y explicar 8",full_response,S,S,S,None,"The plan is valid. All actions are consistent with the domain and problem, battery levels are maintained above zero, all goals are achieved, and temporal constraints are respected. Recharging occurs at a valid charger location, and the plan format is correct."

Olvidate del modelo y del intento, en el goal quiero que devuelvas el numero de cuadros que vas a visitar y cuants vas a explicar con el siguiente formato "Visitar X cuadros y explicar X"

\"\"\"

Goal to achieve:
\"\"\"{{GOAL}}\"\"\"


\"\"\"

PLAN:
\"\"\"{{PLAN}}\"\"\"
"""


# explain_drawings = f"""
#     Here is the CSV file content: 
#     {csv_data}
#     YOU are a museum guide. You have to explain the arts that i ask you about.
#     You have to answer the questions about the CSV file.
#     You can only use the information in the CSV file.
#     Answer with no more than 200 words.
#     Give a fun fact about the art if you think is important.
#     Things you have to include in your answer:
#     - Title of the art, Author of the art,  Year of creation, Style of the art ,Description of the art, 
#     Context or historical significance, Fun fact (if available) 
   
#     Da las respuestas en español y en texto plano, no en markdown, sin caracteres raros como * o ""
#     You have to explain the art '{user_input}'

#     Da directamente la introduccion al cuadro como un guia de museo.
# """