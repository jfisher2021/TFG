(define (problem library_problem)
(:domain library_domain)

(:objects
  shelf1 shelf2 desk corridor door - location
  tiago - robot
  pers_1 - person
  cube - rubik
  book1 - book
)

(:init
  ; Robot initialization
  (robot_at tiago corridor)

  ; Connections
  (connected corridor shelf1)
  (connected corridor shelf2)
  (connected shelf1 corridor)
  (connected shelf2 corridor)
  (connected shelf1 shelf2)
  (connected shelf2 shelf1)
  (connected corridor desk)
  (connected desk corridor)
  (connected door corridor)
  (connected corridor door)

  ; Object locations
  (person_at pers_1 door)
  (rubik_at cube desk)
  (book_at book1 shelf2)
  (not(person_attended pers_1))
)

(:goal
  (and
    (rubik_solved cube)
    (book_found book1)
    (person_attended pers_1)
  )
)

)
