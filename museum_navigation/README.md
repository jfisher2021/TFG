# Museum Navigation - Detalles Técnicos

Este módulo contiene la implementación técnica del sistema de navegación del museo, incluyendo el dominio PDDL, nodos de acción, controlador principal y servicios de voz.

> **📌 Para instalación y ejecución**: Ver el [README principal del TFG](../README.md)

## 🏗️ Arquitectura del Sistema

El sistema está compuesto por varios paquetes ROS 2 que trabajan en conjunto:
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
├── speech_services/            # Servicios de voz
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

