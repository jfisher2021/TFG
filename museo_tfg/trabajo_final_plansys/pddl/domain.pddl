(define (domain library_domain)

(:requirements :strips :fluents :durative-actions :typing)

(:types 
  robot
  location
  rubik 
  book
  person
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (rubik_at ?k - rubik ?l - location)
  (book_at ?b - book ?l - location)
  (person_at ?p - person ?l - location)
  (rubik_solved ?k - rubik)
  (book_found ?b - book)
  (connected ?l1 ?l2 - location)
  (person_attended ?p - person)
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 15)
  :condition (and 
    (at start (robot_at ?r ?from))
    (at start (connected ?from ?to))
  )
  :effect (and 
    (at start (not(robot_at ?r ?from)))
    (at end (robot_at ?r ?to))
  )
)

(:durative-action solve_rubik
  :parameters (?r - robot ?k - rubik ?l - location)
  :duration (= ?duration 10)
  :condition (and
    (over all (robot_at ?r ?l))
    (over all (rubik_at ?k ?l))
  )
  :effect (and
    (at end (rubik_solved ?k))
  )
)

(:durative-action search_book
  :parameters (?r - robot ?b - book ?l - location)
  :duration (= ?duration 20)
  :condition (and 
    (over all (robot_at ?r ?l))
    (over all  (book_at ?b ?l))
  )
  :effect (and 
    (at end (book_found ?b))
  )
)

(:durative-action attend_visitors
  :parameters (?r - robot ?p - person ?d - location)
  :duration (= ?duration 10)
  :condition (and 
    (over all (robot_at ?r ?d))
    (over all (person_at ?p ?d))
  )
  :effect (and 
    (at end (person_attended ?p))
  )
)


)