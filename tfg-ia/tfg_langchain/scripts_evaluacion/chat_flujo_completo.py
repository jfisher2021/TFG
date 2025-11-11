import os
import csv
import json
import sys
from pathlib import Path
from typing import List, TypedDict, Literal, Dict, Any

from dotenv import load_dotenv
from langchain.chat_models import init_chat_model
from openai import OpenAI
from langgraph.graph import END, StateGraph
from pydantic import BaseModel, Field
import time
import ollama

# Añadir la raíz del proyecto al inicio de sys.path
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import prompt_inicial_sin_ejemplos, validate_plan_prompt
load_dotenv(override=True)

pddl_prompt = prompt_inicial_sin_ejemplos

# Elegir el modelo global para la generación del plan
MODEL_TEST = "gpt-5" # Opciones: "deepseek-chat", "gemini-2.5-pro", "gpt-5-mini"

class State(TypedDict):
    goal: str
    plan: str
    validation: List[object]
    time_taken: float

class ResponseFormatter(BaseModel):
    """Always use this tool to structure your response to the user."""

    Format_Valid: Literal["S", "N"] = Field(description="The plan has a valid format (<S/N>)")
    Meets_Goal: Literal["S", "N"] = Field(description="The plan meets the goal (<S/N>) or not")
    Plan_Valid: Literal["S", "N"] = Field(description="The plan is valid (<S/N>) or not (correct format and meets the goal)")
    Goal: str = Field(description="The goal to be achieved")
    Errors: str = Field(description="Any errors found in the plan, if any")
    Comments: str = Field(description="Resume in 10 words the response")
    Tiempo: float = Field(description="Time taken for validation in seconds")


def generate_plan(state: State) -> Dict[str, Any]:
    """
    Función unificada para generar planes PDDL usando diferentes modelos de LLM.
    """
    # Registrar el tiempo de inicio
    tiempo_inicio = time.time()

    goal = state.get("goal", "")
    full_response = ""
    client = None
    is_ollama_model = False
    print(f"INICIANDO GENERACIÓN CON: {MODEL_TEST.upper()}")

    try:
        if "deepseek" in MODEL_TEST.lower():
            selected_model = "deepseek-chat"
            api_key = os.getenv("DEEPSEEK_API_KEY")
            base_url = "https://api.deepseek.com/v1"
            client = OpenAI(base_url=base_url, api_key=api_key)
        elif "gemini" in MODEL_TEST.lower():
            selected_model = MODEL_TEST
            api_key = os.getenv("GEMINI_API_KEY")
            base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"
            client = OpenAI(base_url=base_url, api_key=api_key)
        elif "llama" in MODEL_TEST.lower() or "moonshotai" in MODEL_TEST.lower() or "openai" in MODEL_TEST.lower():
            selected_model = MODEL_TEST
            api_key = os.getenv("GROQ_API_KEY")
            base_url = "https://api.groq.com/openai/v1"
            client = OpenAI(base_url=base_url, api_key=api_key)
        elif "cloud" in MODEL_TEST.lower():
            client = ollama.Client()
            selected_model = MODEL_TEST
            is_ollama_model = True
        elif "gpt" in MODEL_TEST.lower():
            selected_model = MODEL_TEST
            client = OpenAI()
        else:
            raise ValueError(f"Modelo '{MODEL_TEST}' no soportado.")

        print(f"Generando plan PDDL con {selected_model}...")
        print("=" * 50)

        if is_ollama_model:
            response = client.generate(
                model=selected_model,
                prompt=pddl_prompt,
                options={
                    'temperature': 0.1 # Make responses more deterministic MAX 1 and MIN 0
                }
            )
            full_response = response['response']
            print(full_response)
        else:
            response = client.chat.completions.create(
                model=selected_model,
                service_tier="flex",
                messages=[
                    {"role": "user", "content": pddl_prompt},
                ],
                # temperature=0.2,
                stream=False
            )
        
            full_response = response.choices[0].message.content
            print(full_response)

        # Registrar el tiempo de finalización
        tiempo_fin = time.time()

        # Calcular y mostrar el tiempo transcurrido
        tiempo_transcurrido = tiempo_fin - tiempo_inicio
        
    except Exception as e:
        print(f"ERROR EN LLAMADA A LA API ({MODEL_TEST}): {e}")
        return {"goal": goal, "plan": "", "validation": [False, f"Error de API: {e}"], "time_taken": 0.0}

    print_and_save_logs(selected_model, pddl_prompt, full_response)

    return {"goal": goal, "plan": full_response, "validation": [False, ""], "time_taken": tiempo_transcurrido}


def print_and_save_logs(selected_model, pddl_prompt, full_response):
    """Guarda los detalles de la generación del plan en archivos de log."""
    print("\n\n" + "=" * 50)
    print("Guardando logs...")

    # Guardar en archivo de texto
    with open(ROOT_DIR / "log.txt", "a", encoding='utf-8') as f:
        f.writelines(["\nMODEL : ", selected_model, "\nPROMPT:\n", pddl_prompt, "\nRESPONSE:\n", full_response, "\n" + "="*50 + "\n"])

    log_data = {
        "model": selected_model,
        "prompt": pddl_prompt,
        "response": full_response
    }

    # Guardar en archivo JSON
    with open(ROOT_DIR / "log.json", "a", encoding='utf-8') as file:
        json.dump(log_data, file, ensure_ascii=False, indent=2)
        file.write("\n")

    print("Logs guardados en log.txt y log.json")


def save_validation_in_csv(state: State) -> Dict[str, Any]:
    """Valida el plan generado utilizando un modelo con tools y guarda el resultado en CSV."""
    plan = state.get("plan", "")
    goal = state.get("goal", "")
    tiempo = state.get("time_taken", 0.0)
    print("Validando el plan...")

    # Se mantiene la lógica de validación usando LangChain/Google GenAI ya que requieren imports específicos.
    model = init_chat_model("gemini-2.5-flash", model_provider="google_genai")
    model_with_tools = model.bind_tools([ResponseFormatter])
    response = model_with_tools.invoke(validate_plan_prompt.format(PLAN=plan, GOAL=goal))
    
    # Extracción de la llamada a tool
    validation_args = response.tool_calls[0]['args']

    print("\nRESULTADO DE LA VALIDACION...")
    print("=" * 50)
    print(validation_args)
    csv_path = f"{ROOT_DIR}/experimentos_pddl.csv"
    write_header = not os.path.exists(csv_path)

    with open(csv_path, "a", encoding='utf-8', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if write_header:
            writer.writerow(["Modelo","Intento","Goal","Plan_raw","Formato_valido","Cumple_goal","Plan_Valido","Errores","Comentarios","Tiempo"])
        
        # Se guarda el nombre del modelo usado para la generación y los resultados de validación.
        writer.writerow([
            MODEL_TEST, 1, validation_args['Goal'], "full_response", 
            validation_args['Format_Valid'], validation_args['Meets_Goal'], 
            validation_args['Plan_Valid'], validation_args['Errors'], validation_args['Comments'],
            tiempo
        ])

    return {"goal": state.get("goal", ""), "plan": plan, "validation": [validation_args['Plan_Valid'], validation_args['Errors']], "time_taken": tiempo}


if __name__ == "__main__":
    # Configuración del grafo
    graph_builder = StateGraph(State)
    graph_builder.add_node("generate_plan", generate_plan)
    graph_builder.add_node("validate", save_validation_in_csv)

    # Definición del flujo
    graph_builder.set_entry_point("generate_plan")
    graph_builder.add_edge("generate_plan", "validate")
    graph_builder.add_edge("validate", END)

    # Compilando el grafo
    graph = graph_builder.compile()
    # graph.get_graph().draw_mermaid_png(output_file_path="graph.png")
    # print(graph.get_graph().draw_mermaid())

    # Ejecución
    initial_state = {"goal": """
        (:goal
        (and
            (visited tiago monalisa)
            (visited tiago elgrito)
            (visited tiago guernica)
            (visited tiago nocheestrellada)
            (explained_painting monalisa)
            (explained_painting elgrito)
            (explained_painting maestro_aprendiz)
            (explained_painting guernica)
            (explained_painting la_joven_de_la_perla)
            (explained_painting las_meninas)
            (explained_painting el_3_de_mayo_de_1808)
            (explained_painting el_jardin_de_las_delicias)
        
        )
        )""", "plan": "", "validation": [False, ""]}
    # initial_state = {"goal": "Explicar los 31 cuadros", "plan": "", "validation": [False, ""]}
    graph.invoke(initial_state)
