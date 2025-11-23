# TFG - Sistema de Planificación con LLM para Robot Guía de Museo

Este proyecto implementa un sistema de planificación inteligente para un robot guía de museo que utiliza un **Large Language Model (LLM)** para generar planes dinámicamente. El robot navega por un museo virtual, explica obras de arte y gestiona su batería de forma autónoma.

> **📌 Nota sobre la documentación**: Este README ofrece una visión general del proyecto, instrucciones de instalación y ejecución, además de un resumen de la estrctura. Para detalles técnicos profundos sobre el código, consulta los README individuales en cada paquete:
> - 📖 [`museum_navigation/README.md`](museum_navigation/README.md): Dominio PDDL, nodos de acción y controlador
> - 📖 [`llm_planners/README.md`](llm_planners/README.md): Scripts Python y modelos LLM

## 📁 Estructura del Proyecto

Este repositorio contiene el código completo del TFG, organizado de la siguiente manera:

```
TFG/
├── assets/                     # Recursos multimedia
│   ├── cuadros/               # Imágenes de las 31 pinturas del museo
│   └── demo_final_tfg.mp4     # Vídeo demostración del sistema completo
├── llm_planners/              # Experimentos y planificadores con IA
│   ├── langchain_planner/     # Planificador usando LangChain + Gemini
│   ├── ollama_planner/        # Planificador usando Ollama (LLMs locales)
│   ├── pddl/                  # Dominios y problemas PDDL
│   └── scripts/               # Scripts de evaluación y análisis
├── museum_navigation/         # Paquetes ROS2 del sistema del museo
│   ├── museo_plansys/         # Nodos de acción y controlador principal
│   ├── my_llm_plan_solver/    # Plugin LLM para PlanSys2
│   ├── speech_services/       # Servicios TTS y STT
│   └── my_interfaces/         # Definiciones de mensajes/servicios
└── README.md                  # Documentación del proyecto
```

### 📝 Nota Histórica

El código de `llm_planners/` fue originalmente un repositorio independiente que se integró mediante Git Subtree para simplificar el desarrollo. Ahora se mantiene directamente desde este repositorio.


## 🎯 Características Principales

### Innovación: Planificación con LLM
- **Generación dinámica de planes**: El sistema usa un Large Language Model (LLM) para crear planes PDDL personalizados en tiempo real
- **Interacción natural**: El robot pregunta por voz qué pinturas quieres visitar
- **Planes contextuales**: El LLM considera pinturas ya visitadas, nivel de batería y optimiza la ruta

### Sistema Completo de Guía de Museo
- **Navegación autónoma real**: Integración con Nav2 para movimiento del robot Kobuki en Gazebo
- **31 obras de arte**: Colección completa de pinturas famosas con explicaciones detalladas
- **Sistema de voz bidireccional**: 
  - **TTS (Text-to-Speech)**: El robot explica las obras con voz sintetizada
  - **STT (Speech-to-Text)**: Captura peticiones del usuario por voz
- **Gestión inteligente de batería**: Recarga automática cuando es necesario
- **Plugin personalizado**: Implementación de un solver LLM como plugin de PlanSys2


### Recursos Multimedia
- **📁 `assets/cuadros/`**: Imágenes de las 31 pinturas del museo en alta calidad
- **🎥 `assets/demo_final_tfg.mp4`**: Vídeo completo de demostración del sistema funcionando


## 📋 Requisitos Previos

- ROS 2 (Rolling*) Se uso una versión Rolling pero no se ha actualizado a versiones más recientes
- Python 3.8+
- Gazebo
- Nav2
- PlanSys2
- Robot kobuki (simulador)
- Modelo aws_robomaker_bookstore_world (simulado como museo)

## 🚀 Instalación

### 1. Instalar ROS 2 Rolling siguiendo la [guía oficial](https://docs.ros.org/en/rolling/Installation.html).

```bash
sudo apt update
sudo apt upgrade

# Instalacion de Escritorio (Recomendado): ROS, RViz, demos, tutoriales.
sudo apt install ros-rolling-desktop
source /opt/ros/rolling/setup.bash
```

### 2. Clonar Repositorios Necesarios
<details>
  <summary><i>Instalar kobuki (haz click aquí)</i></summary>

  Clona el repositorio en tu espacio de trabajo:
  ```bash
  cd <ros2-workspace>/src
  git clone https://github.com/IntelligentRoboticsLabs/kobuki.git
  ```

  > [!IMPORTANT]
  > Asegúrate de cambiar a la rama `rolling` y de utilizar el commit específico `3063d46ad9bd004c8c6583d600e305d427ee9051` para evitar problemas de compatibilidad. Puedes hacerlo con los siguientes comandos:
  
  ```bash
  git checkout rolling
  git checkout 3063d46ad9bd004c8c6583d600e305d427ee9051
  ```

  Prepara tus repositorios de terceros:
  ```bash
  sudo apt update && sudo apt install ros-dev-tools -y
  cd <ros2-workspace>/src/
  vcs import < kobuki/thirdparty.repos
  ```
  *Por favor, asegúrate de que este último comando no haya fallado. Si ocurre, ejecútalo nuevamente.*

  ### Instalar libusb, libftdi y libuvc
  ```bash
  sudo apt install libusb-1.0-0-dev libftdi1-dev libuvc-dev
  ```

  ### Instalar reglas udev para la cámara astra, kobuki y rplidar
  Cuando conectas un dispositivo de hardware a tu PC, se le asigna `/dev/ttyUSB*`. Este no tendrá los permisos de lectura/escritura necesarios, por lo que no podremos usarlo correctamente. La solución es configurar algunas reglas udev que crean un enlace simbólico con otro nombre (ejemplo: `/dev/ttyUSB0` -> `/dev/kobuki`) y le otorgan los permisos necesarios.
  ```bash
  cd <ros2-workspace>
  sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
  sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
  sudo cp src/ThirdParty/kobuki_ros/60-kobuki.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
</details>
<details>
  <summary><i>Instalar PlanSys2 (haz click aquí)</i></summary>

  Para instalar PlanSys2, ejecuta el siguiente comando:

  ```bash
  sudo apt install ros-<distro>-plansys2-*
  ```

  ```bash
  mkdir -p ~/<ros2-workspace>/src
  cd ~/<ros2-workspace>/src
  git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git
  ```

  > [!IMPORTANT]
  > Asegúrate de utilizar el commit específico `3fc9e946067c75169772851c5d762d323efd5383` para evitar problemas de compatibilidad. Puedes hacerlo con los siguiente comando:

  ```bash
  git checkout 3fc9e946067c75169772851c5d762d323efd5383
  ```

  ```bash
  cd ~/<ros2-workspace>
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
  colcon build --symlink-install
  ```
  
  **Nota**: Asegúrate de reemplazar `<distro>` y `<ros2-distro>` con la versión correspondiente de ROS 2 que estés utilizando.
</details>

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
uv sync
```

**Nota importante**: Asegúrate de que la ruta del entorno virtual en `llm_plan_solver.cpp` coincida con tu instalación.
### 5. Configurar API Keys

El sistema utiliza APIs de LLM para la generación de planes. Actualmente está configurado para usar **Groq** con el modelo `gpt-oss-120b`.

1. **Obtener API key de Groq**:
   - Ve a [https://console.groq.com/keys](https://console.groq.com/keys)
   - Crea una nueva API key

2. **Configurar el archivo `.env`**:
   ```bash
   cd <ros2-workspace>/src/TFG/llm_planners
   echo "GROQ_API_KEY=tu_api_key_aqui" >> .env
   ```

> **Nota**: También puedes usar Gemini o ChatGPT modificando la variable `MODEL_TO_USE` en `get_plan.py` y agregando las respectivas API keys al `.env`.

### 6. Compilar el Workspace

```bash
cd <ros2-workspace>
colcon build --symlink-install
```

> Si tienes problemas de memoria durante la compilación, usa:
> ```bash
> colcon build --symlink-install --parallel-workers 1
> ```

## 🚀 Cómo Ejecutar el Sistema

El sistema funciona de manera completamente integrada y requiere **3 terminales** para su ejecución correcta.

### ⚙️ Prerequisitos

Antes de ejecutar, asegúrate de:
1. Haber compilado el workspace completo: `colcon build --symlink-install`
2. Haber ejecutado `uv sync` en `llm_planners/`
3. Tener configurada la API key de Groq en el archivo `.env` (ver sección 5)

### 📺 Terminal 1: Sistema Principal (Gazebo + PlanSys2 + Servicios)

Este comando lanza todo el sistema integrado en una sola terminal:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch museo_plansys trabajo_final_plansys_launch.py
```

**Este lanzamiento incluye automáticamente:**
- ✅ **Gazebo** con el mundo del museo (bookstore adaptado)
- ✅ **PlanSys2** configurado con el dominio PDDL del museo
- ✅ **Nodos de acción**: move, explain, recharge, welcome
- ✅ **Servicio TTS** (Text-to-Speech) para reproducir explicaciones
- ✅ **Servicio STT** (Speech-to-Text) para capturar peticiones por voz
- ✅ **Robot kobuki** en el entorno simulado

### 🧭 Terminal 2: Navegación Nav2

Lanza el stack de navegación autónoma:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch kobuki navigation_sim.launch.py map:=<ros2-workspace>/install/kobuki/share/kobuki/maps/aws_bookstore.yaml 

```

**Nota**: Reemplaza `<ros2-workspace>` con la ruta al Workspace de ROS 2.

### 🎮 Terminal 3: Controlador del Museo

Una vez que los sistemas anteriores estén activos y cargados:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 run museo_plansys library_controller_node
```

**El controlador ejecutará automáticamente:**
1. ✅ Inicializa el conocimiento del mundo (31 ubicaciones de pinturas + cargador)
2. ✅ Activa el mensaje de bienvenida mediante TTS
3. ✅ Pregunta por voz qué pinturas desea visitar el usuario
4. ✅ Captura la respuesta (por voz con STT o por teclado)
5. ✅ Envía la petición al LLM para generar un plan personalizado
6. ✅ Ejecuta el plan: el robot navega y explica las obras solicitadas
7. ✅ Gestiona la batería automáticamente (recarga cuando es necesario)
8. ✅ Al finalizar, pregunta si desea visitar más pinturas (bucle continuo)

### 🔍 Terminal 4 (Opcional): Monitorización con RQT

Para visualizar el estado interno de PlanSys2:

```bash
rqt
```

En RQT, ve a `Plugins > PlanSys2 Plugin` para inspeccionar:
- 📋 Dominio PDDL cargado
- 🎯 Problema actual
- 📝 Plan generado por el LLM
- ⚙️ Estado de ejecución de acciones

---

## 🔄 Flujo de Ejecución Completo

```
┌─────────────────────────────────────────────────────────────┐
│  1. Sistema arranca (Gazebo + PlanSys2 + Nav2)              │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  2. Controlador inicializa 31 pinturas + ubicaciones        │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  3. TTS: "¿Qué pinturas deseas visitar?"                    │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  4. Usuario responde (voz/STT o texto)                      │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  5. LLM Plugin genera plan PDDL personalizado               │
│     - Considera pinturas solicitadas                        │
│     - Optimiza ruta                                         │
│     - Gestiona batería                                      │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  6. PlanSys2 ejecuta el plan:                               │
│     ├─ move(home, pintura1)    → Nav2 navega                │
│     ├─ explain(pintura1)        → TTS explica               │
│     ├─ move(pintura1, pintura2) → Nav2 navega               │
│     ├─ explain(pintura2)        → TTS explica               │
│     └─ ...                                                  │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  7. Si batería < 20: move(current, home) + recharge()       │ 
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│ 8. El plan ha terminado                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 🤖 Cómo Funciona el Sistema

### Arquitectura General

El proyecto integra múltiples tecnologías para crear un robot guía de museo completamente autónomo:

```
┌─────────────────────────────────────────────────────────────────┐
│                         USUARIO                                 │
│              (Interacción por voz o texto)                      │
└────────────────────────┬────────────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                   CONTROLADOR DEL MUSEO                         │
│            (library_controller_node.cpp)                        │
│  - Gestiona el flujo principal                                  │
│  - Inicializa conocimiento del mundo                            │
│  - Coordina PlanSys2                                            │
└──────┬────────────────────────────────┬─────────────────────────┘
       │                                │
       ▼                                ▼
┌──────────────────┐          ┌─────────────────────────┐
│   PlanSys2       │          │  Servicios de Voz       │
│   ┌──────────┐   │          │  ┌─────────────────┐    │
│   │ Dominio  │   │◄─────────┼──│ TTS Service     │    │
│   │ PDDL     │   │          │  │ (gTTS)          │    │
│   └──────────┘   │          │  └─────────────────┘    │
│   ┌──────────┐   │          │  ┌─────────────────┐    │
│   │ Problema │   │          │  │ STT Service     │    │
│   │ Actual   │   │          │  │ (Whisper)       │    │
│   └──────────┘   │          │  └─────────────────┘    │
│   ┌──────────┐   │          └─────────────────────────┘
│   │ LLM      │   │
│   │ Plugin   │───┼──────────┐
│   └──────────┘   │          │
│   ┌──────────┐   │          ▼
│   │ Executor │   │   ┌─────────────────────────────┐
│   └──────────┘   │   │  LLM Planner (Python)       │
└──────┬───────────┘   │  - LangChain + Ollama       │
       │               │  - Genera planes PDDL       │
       │               │  - Optimiza rutas           │
       ▼               └─────────────────────────────┘
┌──────────────────────────────────────────────┐
│           NODOS DE ACCIÓN                    │
│  ┌────────────┐  ┌──────────────┐            │
│  │ welcome    │  │ move         │            │
│  │ (bienvenida│  │ (Nav2 real)  │            │
│  └────────────┘  └──────────────┘            │
│  ┌────────────┐  ┌──────────────┐            │
│  │ explain    │  │ recharge     │            │
│  │ (TTS)      │  │ (batería)    │            │
│  └────────────┘  └──────────────┘            │
└──────────────┬───────────────────────────────┘
               ▼
┌──────────────────────────────────────────────┐
│        NAVEGACIÓN (Nav2)                     │
│  - Localización (AMCL)                       │
│  - Planificación de rutas                    │
│  - Control del robot                         │
└──────────────┬───────────────────────────────┘
               ▼
┌──────────────────────────────────────────────┐
│        SIMULACIÓN (Gazebo)                   │
│  - Robot kobuki                              │
│  - Mundo del museo (bookstore adaptado)      │
│  - 31 ubicaciones de pinturas + cargador     │
└──────────────────────────────────────────────┘
```

### 🔑 Componentes Clave

#### 1. **Controlador del Museo** (`library_controller_node.cpp`)
El cerebro del sistema que:
- Inicializa las 31 pinturas como instancias PDDL
- Gestiona el ciclo de vida: PLANNING → EXECUTING → PLANNING
- Coordina la interacción entre PlanSys2, servicios de voz y el usuario

#### 2. **Plugin LLM para PlanSys2** (`my_llm_plan_solver/`)
Reemplaza planificadores tradicionales (POPF/TFD) con IA:
- **Entrada**: Dominio PDDL + Problema actual + Pinturas visitadas
- **Proceso**: 
  1. Pregunta al usuario qué pinturas quiere ver (TTS)
  2. Captura respuesta (STT o teclado)
  3. Llama a script Python con LangChain + Ollama
  4. El LLM genera un plan PDDL válido y optimizado
- **Salida**: Plan ejecutable por PlanSys2

#### 3. **Dominio PDDL del Museo** (`domain.pddl`)
Modela el problema de planificación:
- **Tipos**: `robot`, `location`
- **Predicados**: posición del robot, pinturas explicadas, cargadores
- **Funciones**: nivel de batería (0-100)
- **Acciones durativas**:
  - `welcome`: Saludo inicial (1s)
  - `move`: Navegación entre ubicaciones (15s, -20 batería)
  - `explain_painting`: Explicación de obra (15s, -10 batería)
  - `recharge`: Recarga de batería (5s, batería → 100)

#### 4. **Nodos de Acción ROS2**
Implementan las acciones PDDL:
- **move**: Usa BehaviorTree + Nav2 para navegación real
- **explain**: Lee archivo `.txt` y lo reproduce con TTS
- **recharge**: Simula recarga de batería
- **welcome**: Mensaje de bienvenida inicial

#### 5. **Servicios de Voz** (`speech_services/`)
- **TTS**: Convierte texto a voz (Google TTS) para explicaciones
- **STT**: Convierte voz a texto (Whisper) para capturar peticiones

#### 6. **Navegación Nav2**
Stack completo de navegación autónoma:
- Localización AMCL sobre el mapa del museo
- Planificación global y local de rutas
- Control del robot kobuki en tiempo real

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

### 🎨 Pinturas del Museo

El museo virtual cuenta con **31 obras de arte famosas** ubicadas en diferentes salas. Cada pintura tiene:
- 📍 Una ubicación específica en el mapa
- 🖼️ Imagen en alta calidad (`assets/cuadros/`)
- 📝 Explicación detallada en texto (`museo_plansys/explicacion_respuestas/`)
- 🔊 Reproducción por voz mediante TTS

**Lista completa de pinturas:**

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



## 📹 Vídeo de Demostración

**Ver el sistema funcionando completo en `assets/demo_final_tfg.mp4`**

El vídeo muestra:
- ✅ Arranque completo del sistema (Gazebo + PlanSys2 + Nav2)
- ✅ Interacción por voz con el usuario
- ✅ Generación de plan personalizado por el LLM
- ✅ Navegación autónoma del robot entre pinturas
- ✅ Explicaciones mediante TTS
- ✅ Gestión automática de batería
- ✅ Ciclo completo de funcionamiento


[![Demo del Sistema](assets/demo_final_tfg.mp4)](assets/demo_final_tfg.mp4)


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
- **kobuki**: Robot humanoide de PAL Robotics

---

## 👥 Autores

Jonathan Fisher del Río 

Perfil de Github 👨‍: 💻[Github 👨‍💻](https://github.com/jfisher2021)

---

## 📝 Licencia

Este proyecto está bajo licencia Apache 2.0. Ver archivo [LICENSE](LICENSE) para más detalles.

---

**¡Disfruta explorando el museo con inteligencia artificial!** 🎨🤖

