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

# Añadir la raíz del proyecto al inicio de sys.path
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import prompt_inicial_sin_ejemplos, validate_plan_prompt
load_dotenv(override=True)

pddl_prompt = prompt_inicial_sin_ejemplos

# Elegir el modelo global para la generación del plan
MODEL_TEST = "meta-llama/llama-4-maverick-17b-128e-instruct" # Opciones: "deepseek-chat", "gemini-2.5-pro", "gpt-5-mini"

class State(TypedDict):
    goal: str
    plan: str
    validation: List[object]

class ResponseFormatter(BaseModel):
    """Always use this tool to structure your response to the user."""

    Format_Valid: Literal["S", "N"] = Field(description="The plan has a valid format (<S/N>)")
    Meets_Goal: Literal["S", "N"] = Field(description="The plan meets the goal (<S/N>) or not")
    Plan_Valid: Literal["S", "N"] = Field(description="The plan is valid (<S/N>) or not (correct format and meets the goal)")
    Goal: str = Field(description="The goal to be achieved")
    Errors: str = Field(description="Any errors found in the plan, if any")
    Comments: str = Field(description="Resume in 10 words the response")


def generate_plan(state: State) -> Dict[str, Any]:
    """
    Función unificada para generar planes PDDL usando diferentes modelos de LLM.
    """
    goal = state.get("goal", "")
    full_response = ""
    client = None
    
    print(f"INICIANDO GENERACIÓN CON: {MODEL_TEST.upper()}")

    try:
        if "deepseek" in MODEL_TEST.lower():
            selected_model = "deepseek-chat"
            api_key = os.getenv("DEEPSEEK_API_KEY")
            base_url = "https://api.deepseek.com/v1"
            client = OpenAI(base_url=base_url, api_key=api_key,)
        
        elif "gemini" in MODEL_TEST.lower():
            selected_model = "gemini-2.5-pro"
            api_key = os.getenv("GEMINI_API_KEY")
            base_url = "https://generativelanguage.googleapis.com/v1beta"
            client = OpenAI(base_url=base_url, api_key=api_key)
        elif "llama" in MODEL_TEST.lower():
            selected_model = "meta-llama/llama-4-maverick-17b-128e-instruct"
            api_key = os.getenv("GROQ_API_KEY")
            base_url = "https://api.groq.com/openai/v1"
            client = OpenAI(base_url=base_url, api_key=api_key)

        elif "gpt" in MODEL_TEST.lower():
            selected_model = MODEL_TEST
            client = OpenAI()
        
        else:
            raise ValueError(f"Modelo '{MODEL_TEST}' no soportado.")

        print(f"Generando plan PDDL con {selected_model}...")
        print("=" * 50)

        response = client.chat.completions.create(
            model=selected_model,
            messages=[
                {"role": "system", "content": pddl_prompt},
            ],
            # temperature=0.2,
            stream=False
        )
        
        full_response = response.choices[0].message.content
        print(full_response)
        
    except Exception as e:
        print(f"ERROR EN LLAMADA A LA API ({MODEL_TEST}): {e}")
        return {"goal": goal, "plan": "", "validation": [False, f"Error de API: {e}"]}

    print_and_save_logs(selected_model, pddl_prompt, full_response)
    
    return {"goal": goal, "plan": full_response, "validation": [False, ""]}


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
            writer.writerow(["Modelo","Intento","Goal","Plan_raw","Formato_valido","Cumple_goal","Plan_Valido","Errores","Comentarios"])
        
        # Se guarda el nombre del modelo usado para la generación y los resultados de validación.
        writer.writerow([
            MODEL_TEST, 1, validation_args['Goal'], "full_response", 
            validation_args['Format_Valid'], validation_args['Meets_Goal'], 
            validation_args['Plan_Valid'], validation_args['Errors'], validation_args['Comments']
        ])
    
    return {"goal": state.get("goal", ""), "plan": plan, "validation": [validation_args['Plan_Valid'], validation_args['Errors']]}


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

    # Ejecución
    initial_state = {"goal": "Explicar los 31 cuadros", "plan": "", "validation": [False, ""]}
    graph.invoke(initial_state)
