#!/bin/bash

source ~/cuarto/2c/plansis/plansys_ws/install/setup.bash

# for mi_fichero in log_gpt/*instant.txt; do
#   # echo "Procesando: $mi_fichero"
#   # modelo=$(echo "$mi_fichero" | sed -E 's/.*_([^.]*)\..*/\1/')
#   # echo "Modelo: $modelo"
#   num_intento=$(echo "$mi_fichero" | sed -E 's/.*plan([0-9]+)_.*/\1/')
#   # echo "Intento: $num_intento"
#   valid=$(ros2 run popf validate  -t 0.001  pddl/domain.pddl pddl/problem.pddl "$mi_fichero" | egrep "Plan valid")
#   if [ -n "$valid" ]; then
#     echo "El plan es valido"
#     # echo "GPT-5-instant,$num_intento,Visitar 4 cuadros y explicar 8,Plan y formato correctos (todo perfecto),S,S,S,None,"Todo perfecto"" >> experimentos_pddl.csv
#   else
#     echo "El plan no es valido"
#     # echo "GPT-5-instant,$num_intento,Visitar 4 cuadros y explicar 8,Plan incompleto y formato correctos,S,N,N,None,"Plan Invalido"" >> experimentos_pddl.csv

#   fi
# done

# for mi_fichero in log_gpt/*thinking*.txt; do
#   echo "Procesando: $mi_fichero"
#   # modelo=$(echo "$mi_fichero" | sed -E 's/.*_([^.]*)\..*/\1/')
#   # echo "Modelo: $modelo"
#   num_intento=$(echo "$mi_fichero" | sed -E 's/.*plan([0-9]+)_.*/\1/')
#   echo "Intento: $num_intento"
#   valid=$(ros2 run popf validate  -t 0.001  pddl/domain.pddl pddl/problem.pddl "$mi_fichero" | egrep "Plan valid")
#   if [ -n "$valid" ]; then
#     echo "El plan es valido"
#     # echo "GPT-5-thinking,$num_intento,Visitar 4 cuadros y explicar 8,Plan y formato correctos (todo perfecto),S,S,S,None,"Todo perfecto"" >> experimentos_pddl.csv
#   else
#     echo "El plan no es valido"
#     # echo "GPT-5-thinking,$num_intento,Visitar 4 cuadros y explicar 8,Plan incompleto y formato correctos,S,N,N,None,"Plan Invalido"" >> experimentos_pddl.csv

#   fi
# done


for mi_fichero in log_gpt/*instant.txt; do
  # echo "Procesando: $mi_fichero"
  # modelo=$(echo "$mi_fichero" | sed -E 's/.*_([^.]*)\..*/\1/')
  # echo "Modelo: $modelo"
  num_intento=$(echo "$mi_fichero" | sed -E 's/.*plan([0-9]+)_.*/\1/')
  # echo "Intento: $num_intento"
  valid=$(ros2 run popf validate  -t 0.001  pddl/domain.pddl pddl/problem.pddl "$mi_fichero" | egrep "Plan valid")
  if [ -n "$valid" ]; then
    echo "El plan es valido"
    # echo "GPT-5-instant,$num_intento,Visitar 4 cuadros y explicar 8,Plan y formato correctos (todo perfecto),S,S,S,None,"Todo perfecto"" >> experimentos_pddl.csv
  else
    echo "Intento: $num_intento"

    echo "El plan no es valido****************"

    tail -n 30 "$mi_fichero"
    ros2 run popf validate  -t 0.001  pddl/domain.pddl pddl/problem.pddl "$mi_fichero"
    echo "****************"

    # echo "GPT-5-instant,$num_intento,Visitar 4 cuadros y explicar 8,Plan incompleto y formato correctos,S,N,N,None,"Plan Invalido"" >> experimentos_pddl.csv

  fi
done