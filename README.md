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

https://github.com/user-attachments/assets/c668026b-cec0-4327-8386-152151cbb093

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

