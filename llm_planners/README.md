# llm_planners: Evaluación de LLMs para Planificación PDDL

Este proyecto evalúa la capacidad de distintos modelos de lenguaje (LLMs) para generar planes PDDL válidos en un entorno de robot guía de museo, considerando restricciones de batería y optimización de rutas.

## 🌳 Árbol del Proyecto

```
llm_planners/
│
│
├── 🤖 langchain_planner/                    # Implementación con LangChain/LangGraph
│   ├── get_plan.py                      # ⭐ Script principal con StateGraph
│   └── scripts_evaluacion/
│       └── chat_flujo_completo.py       # Flujo completo + validación
├── 🦙 ollama_planner/                       # Implementación con Ollama (local)
│   ├── create_plan.py                   # ⭐ Script principal Ollama
│   ├── logs/                            # Logs JSON/TXT de ejecuciones
│   │   ├── log.json
│   │   ├── log.txt
│   │   └── logv2.txt
│   └── scripts_evaluacion/
│       ├── doble_modelo_correccion.py   # Sistema generador + validador
│       └── validator.py                 # Validación de planes PDDL
│
├── 🎯 pddl/                             # Definiciones PDDL
│   ├── domain.pddl                      # Dominio: acciones, predicados, funciones
│   └── problem.pddl                     # Problema: estado inicial y objetivos
│
├── 🛠️ scripts/                          # Scripts auxiliares
│   ├── create_explain_files.py          # Genera explicaciones con LLMs
│   ├── run_chat_10_times.sh             # Pruebas repetitivas
│   └── analisis_experimentos.py         # Análisis : métricas por modelo
│
├── 📝 LOGS
│   ├── log.txt / log.json               # Logs generales
│   ├── log_gpt.txt                      # Logs específicos GPT
│   └── logs_script*.txt                 # Logs de scripts específicos
│
├── experimentos_pddl.csv            # Resultados de pruebas simples
├── cuadros.csv                      # BD de cuadros 
├── goals.txt                        # Objetivos de prueba en formato PDDL
├── prompts.py                       # Todos los prompts del proyecto
├── utils.py                         # Funciones auxiliares (logs, selección)
├── pyproject.toml                   # Dependencias Python 3.12+
├── 📄 conclusiones.md                   # Conclusiones del TFG
└── README.md                            # Este archivo
```

## 📁 Estructura del Proyecto

### 🎯 Archivos Principales de Evaluación

#### Datos de Experimentos
- **`experimentos_pddl.csv`** - Resultados de pruebas individuales (formato simple)
- **`cuadros.csv`** - Base de datos de cuadros del museo (nombre, autor, estilo, país, etc.)
- **`goals.txt`** - Archivo con diferentes objetivos de prueba en formato PDDL

#### Configuración y Utilidades
- **`prompts.py`** ⭐ - **IMPORTANTE**: Contiene todos los prompts usados para generar y validar planes
- **`utils.py`** - Funciones auxiliares (logging, selección de modelos, etc.)
- **`pyproject.toml`** - Dependencias del proyecto (Python 3.12+)

### 🤖 Directorios de Implementación

#### `langchain_planner/`
Implementación usando LangChain + LangGraph con modelos de OpenAI, Google GenAI y Groq
- **`get_plan.py`** ⭐ - Script principal que genera planes usando un grafo de estados (StateGraph)
  - Usa herramientas (tools) para consultar CSV cuando el goal es en lenguaje natural
  - Configurado actualmente para usar Groq con el modelo `gpt-oss-120b`
- **`scripts_evaluacion/`**
  - `chat_flujo_completo.py` - Flujo completo con validación automática
  - `return_goal_tool_genai.py` - Procesamiento de goals con Google GenAI
  - `return_goal_tools_langchain.py` - Procesamiento de goals con LangChain

#### `ollama_planner/`
Implementación usando Ollama (modelos open-source locales)
- **`create_plan.py`** ⭐ - Script principal para generar planes con Ollama
  - Modelos probados: Llama, Deepseek, Minimax, etc.
- **`logs/`** - Logs de ejecución en JSON y TXT
- **`scripts_evaluacion/`**
  - `doble_modelo_correccion.py` ⭐ - Sistema de doble modelo (generador + validador)
  - `validator.py` - Validación de planes PDDL

#### `pddl/`
Definiciones PDDL del dominio y problema
- **`domain.pddl`** - Dominio PDDL del robot guía (acciones, predicados, funciones)
- **`problem.pddl`** - Problema PDDL con estado inicial y objetivos

#### `scripts/`
Scripts auxiliares de utilidad
- `create_explain_files.py` - Genera explicaciones de cuadros usando LLMs
- `run_chat_10_times.sh` - Script de pruebas repetitivas
- **`analisis_experimentos.py`** - Analizador principal de resultados (v1). Lee `experimentos_pddl.csv` y genera métricas por modelo

### 📝 Archivos de Logs (Experimentales)
- `log.txt`, `log.json`, `log_gpt.txt` - Logs de ejecuciones de pruebas
- `logs_script.txt`, `logs_script_deepseek_razonamiento.txt` - Logs de scripts específicos
- `colclusiones.md` - Documento con conclusiones del TFG

## 🚀 Uso Rápido

### Generar un plan con LangChain (Groq/Gemini/GPT)
```bash
python langchain_planner/get_plan.py "Explica los cuadros españoles"
```

**Nota**: Por defecto usa Groq. Para cambiar el modelo, edita la variable `MODEL_TO_USE` en `get_plan.py`.

### Generar un plan con Ollama
```bash
python ollama_planner/create_plan.py "visited monalisa y explicar guernica"
```

### Analizar resultados
```bash
# Análisis simple
python analisis_experimentos.py

# Análisis con métricas de planner/validator
python analisis_experimentos_v2.py
```

## 📊 Modelos Evaluados

- **OpenAI**: GPT-4o, GPT-4o-mini, GPT-o1-mini
- **Google**: Gemini 2.5 Pro, Gemini 2.5 Flash
- **Deepseek**: Deepseek-chat, Deepseek-r1
- **Meta**: Llama 3.x, Llama 4 Maverick
- **Groq**: Varios modelos optimizados para inferencia
- **Minimax**: M2 (MoE 230B/10B con razonamiento)
- **Otros**: Qwen, Mistral, Phi, etc.

## 🔑 Prompts Principales

Ver `prompts.py` para los prompts completos:
- `prompt_inicial_sin_ejemplos` - Generación de planes (sin ejemplos)
- `prompt_con_3_ejemplos_input_goal` - Generación con few-shot learning
- `validate_plan_prompt` - Validación de planes en formato CSV
- `prompt_get_goal_con_csv` - Extracción de goals con consulta a base de datos

## 📦 Dependencias

Instalar dependencias con uv:

Instala uv si no lo tienes:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Luego, en el directorio llm_planners, ejecuta:

```bash
uv sync
```

## 🔑 Configuración de API Keys

Para usar el script principal `get_plan.py`, necesitas configurar las API keys:

### Groq (Configuración por defecto)

1. Obtén una API key en [https://console.groq.com/keys](https://console.groq.com/keys)
2. Crea un archivo `.env` en el directorio `llm_planners`:

```bash
echo "GROQ_API_KEY=tu_api_key_aqui" >> .env
```

### Otros modelos (opcional)

Para usar Gemini o ChatGPT, añade al `.env`:

```bash
# Para Gemini
GOOGLE_API_KEY=tu_api_key_gemini

# Para ChatGPT  
OPENAI_API_KEY=tu_api_key_openai
```

Y modifica la variable `MODEL_TO_USE` en `get_plan.py`.

## 🔗 Integración con el Sistema ROS2

Este módulo se integra con el sistema ROS2 del museo a través del plugin LLM:

- **Plugin C++**: `../museum_navigation/my_llm_plan_solver/` llama a `get_plan.py`
- **Entrada**: Recibe el estado actual del robot y pinturas visitadas
- **Salida**: Devuelve un plan PDDL válido que PlanSys2 puede ejecutar
- **Comunicación**: El plugin ejecuta el script Python y parsea el resultado

Ver [museum_navigation/README.md](../museum_navigation/README.md) para detalles técnicos de la integración.

## 📦 Dependencias Principales

Las dependencias se gestionan automáticamente con `uv sync`, e incluyen:

- **langchain**: Framework para aplicaciones con LLM
- **langgraph**: Grafos de estados para workflows complejos
- **groq**: Cliente para API de Groq (inferencia rápida)
- **google-genai**: Cliente para modelos Gemini de Google
- **openai**: Cliente para modelos GPT de OpenAI
- **pandas**: Manipulación de datos (CSV de cuadros)
- **python-dotenv**: Gestión de variables de entorno (.env)
