(define (domain library_domain)

(:requirements :strips :fluents :durative-actions :typing)

(:types 
  robot
  location 
)

(:predicates
  (robot_at ?r - robot ?l - location)
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
