# TFG-IA: Evaluación de LLMs para Planificación PDDL

Este proyecto evalúa la capacidad de distintos modelos de lenguaje (LLMs) para generar planes PDDL válidos en un entorno de robot guía de museo, considerando restricciones de batería y optimización de rutas.

## 🌳 Árbol del Proyecto

```
tfg-ia/
│
│
├── 🤖 tfg_langchain/                    # Implementación con LangChain/LangGraph
│   ├── get_plan.py                      # ⭐ Script principal con StateGraph
│   └── scripts_evaluacion/
│       ├── chat_flujo_completo.py       # Flujo completo + validación
│       ├── return_goal_tool_genai.py    # Goals con Google GenAI
│       └── return_goal_tools_langchain.py
│
├── 🦙 tfg_ollama/                       # Implementación con Ollama (local)
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
├── pyproject.toml                   # Dependencias Python 3.13+
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
- **`pyproject.toml`** - Dependencias del proyecto (Python 3.13+)

### 🤖 Directorios de Implementación

#### `tfg_langchain/`
Implementación usando LangChain + LangGraph con modelos de OpenAI y Google GenAI
- **`get_plan.py`** ⭐ - Script principal que genera planes usando un grafo de estados (StateGraph)
  - Usa herramientas (tools) para consultar CSV cuando el goal es en lenguaje natural
- **`scripts_evaluacion/`**
  - `chat_flujo_completo.py` - Flujo completo con validación automática
  - `return_goal_tool_genai.py` - Procesamiento de goals con Google GenAI
  - `return_goal_tools_langchain.py` - Procesamiento de goals con LangChain

#### `tfg_ollama/`
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

### Generar un plan con LangChain (Gemini/GPT)
```bash
python tfg_langchain/get_plan.py "Explica los cuadros españoles"
```

### Generar un plan con Ollama
```bash
python tfg_ollama/create_plan.py "visited monalisa y explicar guernica"
```

### Evaluar con doble modelo (generador + validador)
```bash
python tfg_ollama/scripts_evaluacion/doble_modelo_correccion.py
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

```bash
pip install -e .
```

Requiere: langchain, langgraph, ollama, pandas, google-genai, groq, openai

# TFG-IA: Evaluación de LLMs para Planificación PDDL

Este proyecto evalúa la capacidad de distintos modelos de lenguaje (LLMs) para generar planes PDDL válidos en un entorno de robot guía de museo, considerando restricciones de batería y optimización de rutas.

## 🌳 Árbol del Proyecto

```
tfg-ia/
│
├── 📊 ANÁLISIS Y RESULTADOS
│   ├── analisis_experimentos.py         # Análisis v1: métricas por modelo
│   ├── analisis_experimentos_v2.py      # Análisis v2: métricas planner+validator
│   ├── experimentos_pddl.csv            # Resultados de pruebas simples
│   └── experimentos_pddl_v2.csv         # Resultados con doble modelo
│
├── 📚 DATOS Y CONFIGURACIÓN
│   ├── cuadros.csv                      # BD de cuadros (nombre, autor, estilo, país)
│   ├── goals.txt                        # Objetivos de prueba en formato PDDL
│   ├── prompts.py                       # ⭐ Todos los prompts del proyecto
│   ├── utils.py                         # Funciones auxiliares (logs, selección)
│   └── pyproject.toml                   # Dependencias Python 3.13+
│
├── 🤖 tfg_langchain/                    # Implementación con LangChain/LangGraph
│   ├── get_plan.py                      # ⭐ Script principal con StateGraph
│   └── scripts_evaluacion/
│       ├── chat_flujo_completo.py       # Flujo completo + validación
│       ├── return_goal_tool_genai.py    # Goals con Google GenAI
│       └── return_goal_tools_langchain.py
│
├── 🦙 tfg_ollama/                       # Implementación con Ollama (local)
│   ├── create_plan.py                   # ⭐ Script principal Ollama
│   ├── logs/                            # Logs JSON/TXT de ejecuciones
│   │   ├── log.json
│   │   ├── log.txt
│   │   └── logv2.txt
│   └── scripts_evaluacion/
│       ├── doble_modelo_correccion.py   # ⭐ Sistema generador + validador
│       ├── validator.py                 # Validación de planes PDDL
│       ├── get_goal_from_csv.py         # Extracción de goals desde CSV
│       ├── plan_pruebas.py              # Suite de pruebas automatizadas
│       ├── cuadros.ipynb                # Notebook análisis cuadros
│       └── logs/
│
├── 🎯 pddl/                             # Definiciones PDDL
│   ├── domain.pddl                      # Dominio: acciones, predicados, funciones
│   └── problem.pddl                     # Problema: estado inicial y objetivos
│
├── 🛠️ scripts/                          # Scripts auxiliares
│   ├── create_explain_files.py          # Genera explicaciones con LLMs
│   ├── split_log_gpt.py                 # Procesa logs de GPT
│   ├── trim_plans.py                    # Limpia y formatea planes
│   ├── validate.bash                    # Validación con VAL validator
│   ├── run_chat_10_times.sh             # Pruebas repetitivas
│   └── re_practice.py                   # Experimentos regex
│
├── 📝 LOGS (Experimentales)
│   ├── log.txt / log.json               # Logs generales
│   ├── log_gpt.txt                      # Logs específicos GPT
│   └── logs_script*.txt                 # Logs de scripts específicos
│
├── 📄 colclusiones.md                   # Conclusiones del TFG
└── README.md                            # Este archivo
```

## 📁 Estructura del Proyecto

### 🎯 Archivos Principales de Evaluación

#### Análisis de Resultados
- **`analisis_experimentos.py`** - Analizador principal de resultados (v1). Lee `experimentos_pddl.csv` y genera métricas por modelo
- **`analisis_experimentos_v2.py`** - Analizador avanzado (v2). Lee `experimentos_pddl_v2.csv` con métricas por pareja planner/validator

#### Datos de Experimentos
- **`experimentos_pddl.csv`** - Resultados de pruebas individuales (formato simple)
- **`experimentos_pddl_v2.csv`** - Resultados con sistema de doble modelo (planner + validator)
- **`cuadros.csv`** - Base de datos de cuadros del museo (nombre, autor, estilo, país, etc.)
- **`goals.txt`** - Archivo con diferentes objetivos de prueba en formato PDDL

#### Configuración y Utilidades
- **`prompts.py`** ⭐ - **IMPORTANTE**: Contiene todos los prompts usados para generar y validar planes
- **`utils.py`** - Funciones auxiliares (logging, selección de modelos, etc.)
- **`pyproject.toml`** - Dependencias del proyecto (Python 3.13+)

### 🤖 Directorios de Implementación

#### `tfg_langchain/`
Implementación usando LangChain + LangGraph con modelos de OpenAI y Google GenAI
- **`get_plan.py`** ⭐ - Script principal que genera planes usando un grafo de estados (StateGraph)
  - Usa herramientas (tools) para consultar CSV cuando el goal es en lenguaje natural
  - Soporta: GPT-4, Gemini, Groq
- **`scripts_evaluacion/`**
  - `chat_flujo_completo.py` - Flujo completo con validación automática
  - `return_goal_tool_genai.py` - Procesamiento de goals con Google GenAI
  - `return_goal_tools_langchain.py` - Procesamiento de goals con LangChain

#### `tfg_ollama/`
Implementación usando Ollama (modelos open-source locales)
- **`create_plan.py`** ⭐ - Script principal para generar planes con Ollama
  - Modelos probados: Llama, Deepseek, Minimax, etc.
- **`logs/`** - Logs de ejecución en JSON y TXT
- **`scripts_evaluacion/`**
  - `doble_modelo_correccion.py` ⭐ - Sistema de doble modelo (generador + validador)
  - `validator.py` - Validación de planes PDDL
  - `get_goal_from_csv.py` - Extracción de goals desde CSV
  - `plan_pruebas.py` - Suite de pruebas automatizadas
  - `cuadros.ipynb` - Notebook de análisis de datos de cuadros

#### `pddl/`
Definiciones PDDL del dominio y problema
- **`domain.pddl`** - Dominio PDDL del robot guía (acciones, predicados, funciones)
- **`problem.pddl`** - Problema PDDL con estado inicial y objetivos

#### `scripts/`
Scripts auxiliares de utilidad
- `create_explain_files.py` - Genera explicaciones de cuadros usando LLMs
- `split_log_gpt.py` - Procesa y divide logs de GPT
- `trim_plans.py` - Limpia y formatea planes PDDL
- `validate.bash` - Script de validación de planes (requiere VAL validator)
- `run_chat_10_times.sh` - Script de pruebas repetitivas
- `re_practice.py` - Experimentos con expresiones regulares

### 📝 Archivos de Logs (Experimentales)
- `log.txt`, `log.json`, `log_gpt.txt` - Logs de ejecuciones de pruebas
- `logs_script.txt`, `logs_script_deepseek_razonamiento.txt` - Logs de scripts específicos
- `colclusiones.md` - Documento con conclusiones del TFG

## 🚀 Uso Rápido

### Generar un plan con LangChain (Gemini/GPT)
```bash
python tfg_langchain/get_plan.py "Explica los cuadros españoles"
```

### Generar un plan con Ollama
```bash
python tfg_ollama/create_plan.py "visited monalisa y explicar guernica"
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

```bash
pip install -e .
```

Requiere: langchain, langgraph, ollama, pandas, google-genai, groq, openai