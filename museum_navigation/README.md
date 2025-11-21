# TFG - Sistema de Planificación con LLM para Robot Guía de Museo

Este proyecto implementa un sistema de planificación inteligente para un robot guía de museo que utiliza un **Large Language Model (LLM)** para generar planes dinámicamente. El robot navega por un museo virtual, explica obras de arte y gestiona su batería de forma autónoma.

## 🎯 Características Principales

- **Planificación con LLM**: Generación de planes mediante inteligencia artificial (usando LangChain + Ollama)
- **Navegación autónoma**: Integración con Nav2 para navegación real del robot TIAGo
- **Sistema de voz**: Text-to-Speech (TTS) y Speech-to-Text (STT) para interacción
- **Gestión de batería**: El robot recarga automáticamente cuando es necesario
- **31 obras de arte**: Explicaciones detalladas de pinturas famosas
- **Plugin personalizado**: Implementación de un solver LLM para PlanSys2


## 📋 Requisitos Previos

- ROS 2 (Humble/Rolling)
- Python 3.8+
- Gazebo
- Nav2
- PlanSys2
- Robot TIAGo (simulador)
- Modelo aws_robomaker_bookstore_world (adaptado como museo)

## 🚀 Instalación

### 1. Dependencias del Sistema

```bash
source /opt/ros/<ROS-DISTRO>/setup.bash
sudo apt update
sudo apt install ros-dev-tools -y
```

### 2. Clonar Repositorios Necesarios

```bash
cd <ros2-workspace>/src
# TIAGo Harmonic
git clone https://github.com/Tiago-Harmonic/tiago_harmonic.git
# Mundo del museo (bookstore modificado)
git clone https://github.com/<tu-repo>/aws_robomaker_bookstore_world.git
```

### 3. Instalar Dependencias con rosdep

```bash
cd <ros2-workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Configurar Entorno Python para LLM

Este proyecto requiere un entorno Python con LangChain y Ollama:

```bash
cd <ros2-workspace>/src/TFG/llm_planners
python3 -m venv .venv
source .venv/bin/activate
pip install langchain langchain-ollama openai
```

**Nota importante**: Asegúrate de que la ruta del entorno virtual en `llm_plan_solver.cpp` coincida con tu instalación.

### 5. Compilar el Workspace

```bash
cd <ros2-workspace>
colcon build --symlink-install
```

> Si tienes problemas de memoria durante la compilación, usa:
> ```bash
> colcon build --symlink-install --parallel-workers 1
> ```

## 🎮 Ejecución del Sistema

El sistema requiere **2 terminales** para funcionar correctamente:

### Terminal 1: Sistema Principal (Gazebo + PlanSys2 + Servicios)

Este comando lanza todo el sistema integrado:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch museo_plansys trabajo_final_plansys_launch.py
```

**Este lanzamiento incluye:**
- ✅ Gazebo con el mundo del museo (bookstore.world)
- ✅ PlanSys2 con el dominio PDDL personalizado
- ✅ Nodos de acción (move, explain, recharge, welcome)
- ✅ Servicio TTS (Text-to-Speech)
- ✅ Servicio STT (Speech-to-Text)

### Terminal 2: Navegación Nav2

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch tiago_harmonic navigation_sim.launch.py map:=<path-to-museum-map>
```

### Terminal 3: Controlador del Museo

Una vez que los sistemas anteriores estén activos:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 run museo_plansys library_controller_node
```

**El controlador automáticamente:**
1. Inicializa el conocimiento del mundo (31 ubicaciones de pinturas)
2. Solicita al LLM que genere un plan de visita
3. Ejecuta el plan: el robot navega y explica las obras
4. Reproduce las explicaciones mediante TTS

### 🔍 Monitorización (Opcional)

Para visualizar el estado de PlanSys2:

```bash
rqt
```

En RQT, ve a `Plugins > Planning System` para ver el estado del dominio, problema, plan y ejecución.

---

## 📐 Dominio PDDL

El dominio PDDL modela un museo donde el robot puede moverse entre ubicaciones, explicar pinturas y recargar su batería.

**Archivo**: [`museo_plansys/pddl/domain.pddl`](museo_plansys/pddl/domain.pddl)

### Tipos

```pddl
(:types 
  robot
  location 
)
```

### Predicados

- `(robot_at ?r - robot ?l - location)`: Posición actual del robot
- `(explained_painting ?p - location)`: Indica si una pintura ha sido explicada
- `(can_start ?r - robot)`: Permite iniciar acciones
- `(initial_state ?r - robot)`: Estado inicial del robot
- `(visited ?r - robot ?l - location)`: Marca ubicaciones visitadas
- `(charger_at ?wp - location)`: Ubicación del punto de recarga

### Funciones (Fluents)

- `(battery ?r - robot)`: Nivel de batería del robot (0-100)

### Acciones Durativas

#### 1. **start_welcome**
```pddl
:duration 1 segundo
:condition initial_state
:effect can_start (permite comenzar el recorrido)
```

#### 2. **move**
```pddl
:duration 15 segundos
:condition batería >= 20, can_start, robot_at origen
:effect 
  - robot_at destino
  - visited destino
  - batería -= 20
```

#### 3. **explain_painting**
```pddl
:duration 15 segundos
:condition batería >= 10, robot_at pintura, can_start
:effect 
  - explained_painting
  - batería -= 10
```

#### 4. **recharge**
```pddl
:duration 5 segundos
:condition batería <= 100, robot_at cargador, charger_at cargador
:effect batería = 100
```

### 🎨 Obras de Arte en el Museo

El museo cuenta con **31 pinturas famosas**:

1. Mona Lisa
2. La Noche Estrellada
3. El Grito
4. Guernica
5. La Joven de la Perla
6. Las Meninas
7. El 3 de Mayo de 1808
8. El Jardín de las Delicias
9. Las Tres Gracias
10. La Rendición de Breda
... y 21 más

Cada pintura tiene su propia ubicación (`location`) y el robot puede navegar hasta ella y explicarla.

---

## 🤖 Implementación de Acciones

Las acciones PDDL se implementan como nodos de ROS 2 en C++:

**Directorio**: [`museo_plansys/src/`](museo_plansys/src/)

### Acciones Implementadas

#### 1. **move_action** (Navegación Real con Nav2)
- **Archivo**: Usa `plansys2_bt_actions` con BehaviorTree
- **XML**: [`behavior_trees_xml/move.xml`](museo_plansys/behavior_trees_xml/move.xml)
- **Funcionalidad**: Navegación real del robot usando Nav2
- **Duración**: Variable según distancia

#### 2. **explain_action_node.cpp**
- **Funcionalidad**: 
  - Lee el archivo de explicación de la pintura desde `explicacion_respuestas/*.txt`
  - Llama al servicio TTS para reproducir la explicación
  - Marca la pintura como explicada
- **Duración**: 15 segundos

#### 3. **recharge_action_node.cpp**
- **Funcionalidad**: Recarga la batería del robot al 100%
- **Condición**: El robot debe estar en la ubicación `home` (donde está el cargador)
- **Duración**: 5 segundos

#### 4. **welcome_action_node.cpp**
- **Funcionalidad**: Acción inicial de bienvenida
- **Efecto**: Habilita `can_start` para permitir otras acciones
- **Duración**: 1 segundo

### 🌳 BehaviorTrees

La acción de movimiento utiliza un BehaviorTree XML que integra Nav2:

```xml
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Nav2Client name="nav2_client" goal="{waypoint}"/>
        </Sequence>
    </BehaviorTree>
</root>
```

---

## 🧠 Plugin LLM Plan Solver

**Lo más innovador del proyecto**: Implementación de un planificador basado en LLM que reemplaza a POPF/TFD.

**Paquete**: [`my_llm_plan_solver/`](my_llm_plan_solver/)

### Funcionamiento

1. **Entrada**: El solver recibe el dominio PDDL y el problema actual
2. **Interacción de voz**:
   - Usa TTS para preguntar al usuario qué pinturas quiere visitar
   - (Opcional) Usa STT para capturar la respuesta por voz
3. **Generación de plan**:
   - Llama a un script Python que usa LangChain + Ollama
   - El LLM genera un plan PDDL válido considerando:
     - Las pinturas solicitadas
     - Las pinturas ya visitadas
     - La gestión de batería
     - Optimización de la ruta
4. **Parsing**: Convierte el plan textual en mensajes `plansys2_msgs::msg::Plan`
5. **Ejecución**: PlanSys2 ejecuta el plan generado

### Archivos Clave

- **C++**: [`llm_plan_solver.cpp`](my_llm_plan_solver/src/my_llm_plan_solver/llm_plan_solver.cpp)
  - Plugin que hereda de `PlanSolverBase`
  - Integra servicios TTS/STT
  - Ejecuta script Python y parsea resultado
  
- **Python**: `llm_planners/langchain_planner/get_plan.py` (repositorio externo)
  - Usa LangChain para interactuar con el LLM
  - Genera planes PDDL válidos
  - Considera contexto e historial de visitas

### Ventajas sobre Planificadores Tradicionales

- ✅ **Flexibilidad**: Entiende lenguaje natural
- ✅ **Contextual**: Considera preferencias del usuario
- ✅ **Adaptativo**: Puede ajustar planes según feedback
- ✅ **Explicable**: Puede justificar sus decisiones

---

## 🎙️ Servicios de Voz

**Paquete**: [`speech_services/`](speech_services/)

### Text-to-Speech (TTS)

- **Nodo**: `tts_service.py`
- **Servicio**: `/tts_service` (TextToSpeech)
- **Tecnología**: gTTS (Google Text-to-Speech)
- **Uso**: Reproduce explicaciones de pinturas

```bash
# Probar manualmente
ros2 service call /tts_service my_interfaces/srv/TextToSpeech "{text: 'Bienvenido al museo'}"
```

### Speech-to-Text (STT)

- **Nodo**: `stt_service.py`
- **Servicio**: `/stt_service` (SpeechToText)
- **Tecnología**: OpenAI Whisper
- **Idioma**: Español (`language='es'`)
- **Uso**: Captura peticiones de pinturas por voz

```bash
# Probar manualmente
ros2 service call /stt_service my_interfaces/srv/SpeechToText
```

---

## 🎯 Controlador del Museo

**Archivo**: [`library_controller_node.cpp`](museo_plansys/src/library_controller_node.cpp)

El controlador orquesta todo el sistema:

1. **Inicialización**: Carga las 31 ubicaciones de pinturas
2. **Bucle principal**:
   ```
   PLANNING → EXECUTING → PLANNING → ...
   ```
3. **Estados**:
   - `PLANNING`: Solicita nuevo plan al LLM solver
   - `EXECUTING`: Ejecuta el plan generado
   - `ENDING`: Finaliza cuando se completan todas las metas

### Flujo de Ejecución

```
┌─────────────────┐
│  Inicializar    │
│  conocimiento   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  LLM genera     │
│  plan           │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  PlanSys2       │
│  ejecuta plan   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  ¿Más pinturas? │
│  → Loop         │
└─────────────────┘
```

---

## 🏗️ Estructura del Proyecto

```
museum_navigation/
├── museo_plansys/              # Paquete principal ROS 2
│   ├── pddl/
│   │   ├── domain.pddl         # Dominio PDDL del museo
│   │   └── problem.pddl        # Problema ejemplo
│   ├── src/
│   │   ├── explain_action_node.cpp
│   │   ├── move_fake_action_node.cpp
│   │   ├── recharge_action_node.cpp
│   │   ├── welcome_action_node.cpp
│   │   └── library_controller_node.cpp
│   ├── behavior_trees_xml/
│   │   └── move.xml            # BT para navegación Nav2
│   ├── explicacion_respuestas/ # Textos de explicación (31 archivos)
│   └── launch/
│       └── trabajo_final_plansys_launch.py
│
├── my_llm_plan_solver/         # Plugin LLM para PlanSys2
│   ├── include/my_llm_plan_solver/
│   │   └── llm_plan_solver.hpp
│   └── src/my_llm_plan_solver/
│       └── llm_plan_solver.cpp # Solver que llama al LLM
│
├── speech_services/              # Servicios de voz
│   └── speech_services/src/
│       ├── tts_service.py      # Text-to-Speech
│       └── stt_service.py      # Speech-to-Text
│
└── my_interfaces/              # Definiciones de servicios
    └── srv/
        ├── TextToSpeech.srv
        └── SpeechToText.srv
```

---

## 🔧 Resolución de Problemas

### El LLM no genera planes

**Problema**: El script Python no ejecuta o falla.

**Solución**:
1. Verifica que el entorno virtual esté activado y tenga las dependencias:
   ```bash
   source <path>/llm_planners/.venv/bin/activate
   pip list | grep langchain
   ```

2. Verifica la ruta del intérprete Python en `llm_plan_solver.cpp` (línea ~177):
   ```cpp
   std::string command = "/ruta/correcta/.venv/bin/python /ruta/correcta/get_plan.py " + ...
   ```

3. Verifica que Ollama esté ejecutándose:
   ```bash
   ollama list
   ollama run llama3  # o el modelo que uses
   ```

### Nav2 no navega

**Problema**: El robot no se mueve a los waypoints.

**Solución**:
1. Verifica que el mapa esté cargado correctamente
2. Usa RViz para ver los costmaps y la localización
3. Asegúrate de que los waypoints en el código coincidan con coordenadas del mapa

### Servicios TTS/STT no responden

**Problema**: Los servicios no están disponibles.

**Solución**:
```bash
# Verifica que los nodos estén activos
ros2 node list | grep tts
ros2 node list | grep stt

# Verifica los servicios
ros2 service list | grep tts_service
ros2 service list | grep stt_service
```

### Errores de compilación

**Problema**: Faltan includes o dependencias.

**Solución**:
```bash
# Reinstala dependencias
cd <ros2-workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Limpia y recompila
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 📊 Configuración Avanzada

### Modificar la Lista de Pinturas por Defecto

En `llm_plan_solver.cpp` (líneas 159-166), puedes cambiar las pinturas que se incluyen en el prompt al LLM:

```cpp
const std::vector<std::string> visited_tokens = {
  "autorretrato_con_collar_de_espinas",
  "el_carnaval_del_arlequin",
  "nighthawks",
  // Añade más pinturas aquí
};
```

### Ajustar Parámetros de Batería

En `domain.pddl`, modifica los costos de batería:

```pddl
(:durative-action move
  ...
  :effect (at start (decrease (battery ?r) 20))  ; Cambiar este valor
)
```

### Personalizar Explicaciones

Edita los archivos en `explicacion_respuestas/*.txt` para cambiar las explicaciones de las pinturas.

---


## 📹 Vídeo de Demostración

Ver el sistema funcionando completo:


https://github.com/user-attachments/assets/e53534b6-c863-4cac-b03d-f919722aff35


---

## 📚 Referencias y Tecnologías Utilizadas

- **ROS 2**: Framework de robótica
- **PlanSys2**: Sistema de planificación basado en PDDL
- **Nav2**: Stack de navegación autónoma
- **LangChain**: Framework para aplicaciones con LLM
- **Ollama**: Servicio local de modelos LLM
- **OpenAI Whisper**: Modelo de Speech-to-Text
- **gTTS**: Google Text-to-Speech
- **Gazebo**: Simulador de robótica
- **TIAGo**: Robot humanoide de PAL Robotics

---

## 👥 Autores

Trabajo Final de Grado - Planificación de Sistemas Robóticos
Universidad Rey Juan Carlos

---

## 📝 Licencia

Este proyecto está bajo licencia Apache 2.0. Ver archivo [LICENSE](LICENSE) para más detalles.

---

## 🆘 Soporte

Para problemas o preguntas:
1. Revisa la sección de **Resolución de Problemas**
2. Verifica los logs de ROS 2: `ros2 topic echo /rosout`
3. Consulta la documentación de [PlanSys2](https://plansys2.github.io/)
4. Abre un issue en el repositorio

---

**¡Disfruta explorando el museo con inteligencia artificial!** 🎨🤖


## Ejercicio 4 - Planner
Utilizar un planificador distinto a POPF o TFD para generar los planes. Para conseguir esto es necesario implementar un plugin para poder llamar al planificador elegido desde PlanSys2. Este plugin se debe crear en un **paquete de ROS 2 aparte**, y consistirá en una clase que herede de [PlanSolverBase](https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_core/include/plansys2_core/PlanSolverBase.hpp). Se puede utilizar la implementación del [plugin de POPF](https://github.com/PlanSys2/ros2_planning_system/tree/rolling/my_llm_plan_solver) como referencia.

**Nota:** Si es posible, se deberá implementar un plugin para el planificador escogido por el grupo en el trabajo de PDDL. El planificador deberá soportar al menos `durative-actions`, por lo que si se escogió un planificador no compatible tendréis que elegir otro para implementar el plugin de PlanSys2.


Plugin escogido y detalles de cómo poder usarlo en plansys:

*[Respuesta]*


## Vídeo final
Para finalizar, se debe incluir un vídeo del sistema funcionando, en el que se pueda apreciar el desarrollo realizado.


### COMANDOS EJECUCIÓN FINAL

Para que el programa funcione correctamente, se deben ejecutar los siguientes comandos:

Terminal 1:

Lanzamos nuestro controlador y a la vez lanzamos gazebo con el mundo de la biblioteca:
```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch museo_plansys museo_plansys_launch.py 
```

Terminal 2:

Lanzamos el sistema de NAV2:
```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch kobuki navigation_sim.launch.py map:=src/kobuki/maps/aws_bookstore.yaml 
```

Terminal 3:

Lanzamos el los nodos:
```bash
cd <ros2-workspace>
source install/setup.bash
ros2 run museo_plansys library_controller_node 
```

Terminal 4 (opcional):

Para ver los plugins de PlanSys2:
```bash
rqt
```


**Vídeo**


https://github.com/user-attachments/assets/e53534b6-c863-4cac-b03d-f919722aff35




## Presentación
Se realizará una presentación de 10min del trabajo en clase y deberéis añadir también al repositorio los materiales (slides, vídeos, etc.) utilizados en la presentación.