import csv
import json
import sys
from pathlib import Path
from typing import List, TypedDict, Literal

from dotenv import load_dotenv
from google import genai
from google.genai import types
from langchain.chat_models import init_chat_model
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.graph import END, StateGraph
from pydantic import BaseModel, Field

# Añadir la raíz del proyecto (carpeta 'tfg') al inicio de sys.path que funcionen los imports
ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


from prompts import prompt_inicial_sin_ejemplos, validate_plan_prompt
load_dotenv(override=True)

pddl_prompt = prompt_inicial_sin_ejemplos

# Elegir el modelo según una condición

class State(TypedDict):
    goal: str
    plan: str
    # validation será una lista [bool, str]: [es_valido, razon]
    validation: List[object]

class ResponseFormatter(BaseModel):
    """Always use this tool to structure your response to the user."""
    Format_Valid: Literal["S", "N"] = Field(description="The plan has a valid format (<S/N>)")
    Meets_Goal: Literal["S", "N"] = Field(description="The plan meets the goal (<S/N>) or not")
    Plan_Valid: Literal["S", "N"] = Field(description="The plan is valid (<S/N>) or not (correct format and meets the goal)")
    Goal: str = Field(description="The goal to be achieved")
    Errors: str = Field(description="Any errors found in the plan, if any")
    Comments: str = Field(description="Resume in 10 words the response")

def gemini_chat(state: State):
    print("USANDO GEMINI-2.5-PRO")
    selected_model = "gemini-2.5-pro"
    client = genai.Client()
    response = client.models.generate_content(
        model=selected_model, contents=pddl_prompt,
        config=genai.types.GenerateContentConfig(
            temperature=0.0,
        ),
    )

    print("\nGenerando plan PDDL...")
    print("=" * 50)
    print(response.text)
    full_response = response.text
    print_and_save_logs(selected_model, pddl_prompt, full_response)
    # Inicializar validation como [False, ""]; la validación real la hará el nodo validate
    return {"goal": state.get("goal", ""), "plan": full_response, "validation": [False, ""]}

def chatgpt_chat(state: State):
    # Crear el modelo de LangChain con OpenAI
    print("USANDO GPT-4O-MINI")
    selected_model = "gpt-4o-mini"
    model = ChatOpenAI(
        model=selected_model,
        temperature=1.0,
    )
    full_response = ""
    for chunk in model.stream(pddl_prompt):
        print(chunk.content, end='', flush=True)
        full_response += chunk.content

    print("\nGenerando plan PDDL...")
    print("=" * 50)
    print_and_save_logs(selected_model, pddl_prompt, full_response)
    return {"goal": state.get("goal", ""), "plan": full_response, "validation": [False, ""]}


def print_and_save_logs(selected_model, pddl_prompt, full_response):

    print("\n\n" + "=" * 50)
    print("Guardando logs...")

    # Guardar en archivo de texto
    with open(ROOT_DIR / "log.txt", "a", encoding='utf-8') as f:
        f.writelines(["\nMODEL : ", selected_model, "\nPROMPT:\n", pddl_prompt, "\nRESPONSE:\n", full_response, "\n" + "="*50 + "\n"])

    # Guardar en archivo JSON
    log_data = {
        "model": selected_model,
        "prompt": pddl_prompt,
        "response": full_response
    }

    with open(ROOT_DIR / "log.json", "a", encoding='utf-8') as file:
        json.dump(log_data, file, ensure_ascii=False, indent=2)
        file.write("\n")

    print("Logs guardados en log.txt y log.json")


def save_validation_in_csv(state: State):
    plan = state.get("plan", "")
    goal = state.get("goal", "")
    print("Validando el plan...")

    # EN CASO DE QUERER USAR EL MODELO DE OPENAI

    # print("USANDO GPT-4O-MINI")
    # selected_model = "gpt-4o-mini"
    # model = ChatOpenAI(
    #     model=selected_model,
    #     temperature=1.0,
    # )
    # model_with_tools = model.bind_tools([ResponseFormatter])
    # response = model_with_tools.invoke(validate_plan_prompt.format(PLAN=plan, GOAL=goal))

    model = init_chat_model("gemini-2.5-flash", model_provider="google_genai")
    model_with_tools = model.bind_tools([ResponseFormatter])
    response = model_with_tools.invoke(validate_plan_prompt.format(PLAN=plan, GOAL=goal))
    print("\nRESULTAADO DE LA VALIDACION...")
    print("=" * 50)
    print(response)
    print(f"El Plan es válido?: {response.tool_calls[0]}")

    full_response = response.content
    # Inicializar validation como [False, ""]; la validación real la hará el nodo validate
    csv_path = f"{ROOT_DIR}/experimentos_pddl.csv"
    write_header = False
    try:
        # Si el archivo no existe, escribimos cabecera
        import os
        write_header = not os.path.exists(csv_path)
    except Exception:
        write_header = False

    with open(csv_path, "a", encoding='utf-8', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if write_header:
            writer.writerow(["Modelo","Intento","Goal","Plan_raw","Formato_valido","Cumple_goal","Plan_Valido","Errores","Comentarios"])
        writer.writerow([response.response_metadata["model_name"], 1, response.tool_calls[0]['args']['Goal'], "full_response", response.tool_calls[0]['args']['Format_Valid'], response.tool_calls[0]['args']['Meets_Goal'], response.tool_calls[0]['args']['Plan_Valid'], response.tool_calls[0]['args']['Errors'], response.tool_calls[0]['args']['Comments']])
    return {"goal": state.get("goal", ""), "plan": full_response, "validation": ["True", "yes"]}

if __name__ == "__main__":

    # Añadiendo los nodos
    graph_builder = StateGraph(State)
    # graph_builder.add_node("chatgpt", chatgpt_chat)
    graph_builder.add_node("gemini", gemini_chat)
    graph_builder.add_node("validate", save_validation_in_csv)

    # Creacion del grafo
    graph_builder.set_entry_point("gemini")
    graph_builder.add_edge("gemini", "validate")
    graph_builder.add_edge("validate", END)

    # Compilando el grafo
    graph = graph_builder.compile()

    # graph.get_graph().draw_mermaid_png(output_file_path="graph.png")
    initial_state = {"goal": "", "plan": "", "validation": [False, ""]}
    graph.invoke(initial_state)

