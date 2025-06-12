(define (problem library_problem)
(:domain library_domain)

(:objects
  home monalisa nocheestrellada elgrito dibejo guernica la_joven_de_la_perla las_meninas el_3_de_mayo_de_1808 
  el_jardin_de_las_delicias las_tres_gracias la_rendicion_de_breda el_nacimiento_de_venus
   la_creacion_de_adan la_ultima_cena la_libertad_guiando_al_pueblo el_hijo_del_hombre american_gothic 
   la_persistencia_de_la_memoria la_ronda_de_noche impresion,_sol_naciente banistas_en_asnieres 
   saturno_devorando_a_su_hijo whistlers_mother la_gran_ola_de_kanagawa el_beso autorretrato_con_collar_de_espinas 
   el_carnaval_del_arlequin nighthawks retrato_de_adele_bloch-bauer_i campbells_soup_cans composition_viii - location

  tiago - robot
  ; pers_1 - person
)

(:init
  ; Robot initialization
  (robot_at tiago home)
  (initial_state tiago)

  ; Object locations
  (= (battery tiago) 100)
  ; (person_at pers_1 guernica)
  ; (not(person_attended pers_1))
  (charger_at home)
)

(:goal
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

)
