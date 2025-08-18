from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI
from google import genai
from google.genai import types
from dotenv import load_dotenv
import json
import sys
from langgraph.graph import END, StateGraph
from typing import Annotated, List, TypedDict, Union
import csv



from langchain_ollama import ChatOllama
import ollama

sys.path.append(sys.path[0] + "/..")
import prompts 
load_dotenv()

pddl_prompt = prompts.prompt_inicial_sin_ejemplos
# Elegir el modelo según una condición


class State(TypedDict):
    goal: str
    plan: str
    # validation será una lista [bool, str]: [es_valido, razon]
    validation: List[object]


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
    with open("log.txt", "a", encoding='utf-8') as f:
        f.writelines(["\nMODEL : ", selected_model, "\nPROMPT:\n", pddl_prompt, "\nRESPONSE:\n", full_response, "\n" + "="*50 + "\n"])

    # Guardar en archivo JSON
    log_data = {
        "model": selected_model,
        "prompt": pddl_prompt,
        "response": full_response
    }

    with open("log.json", "a", encoding='utf-8') as file:
        json.dump(log_data, file, ensure_ascii=False, indent=2)
        file.write("\n")

    print("Logs guardados en log.txt y log.json")

def validate_plan(state: State):
    # Aquí podrías implementar una validación del plan PDDL
    # Por ejemplo, verificar que las acciones son válidas y que la batería se maneja correctamente
    # Crear el modelo de LangChain con OpenAI

    # Validar el plan usando Ollama
    client = ollama.Client()
    models = ollama.list()
    print("Modelos disponibles:\n")
    for idx, model in enumerate(models['models']):
        print(f"{idx + 1}: {model['model']}")
    try:
        model_index = int(input("Selecciona el número del modelo que quieres usar: ")) - 1
    except Exception:
        # Si no es posible leer input (ej. ejecución no interactiva), usar el primer modelo
        model_index = 0
    # Asegurar índice válido
    if model_index < 0 or model_index >= len(models['models']):
        model_index = 0
    selected_model = models['models'][model_index]['model']
    print(f"Has seleccionado el modelo: {selected_model}")
    model = ChatOllama(
        model=selected_model,
        temperature=0.0,
    )
    plan = state.get("plan", "")
    full_response = ""
    for chunk in model.stream(prompts.validate_plan_prompt.format(PLAN=plan)):
        print(chunk.content, end='', flush=True)
        full_response += chunk.content
    print("\nGenerando validación...")
    print("=" * 50)

    # Heurística simple para determinar si la validación es positiva o negativa
    text_lower = full_response.lower()
    negative_keywords = ["invalid", "no válido", "no es válido", "no valido", "incorrect", "erróneo", "erroneo", "no cumple", "falla"]
    is_valid = True
    for kw in negative_keywords:
        if kw in text_lower:
            is_valid = False
            break

    validation_list = [is_valid, full_response]
    # incluir modelo seleccionado en el estado para registro si es útil
    return {"goal": state.get("goal", ""), "plan": plan, "validation": validation_list, "model": selected_model}


def save_logs_node(state: State):
    """Nodo final: guarda prompt, goal, plan y validation (bool + razon) en CSV y JSON."""
    prompt_text = pddl_prompt
    goal = state.get("goal", "")
    plan = state.get("plan", "")
    validation = state.get("validation", [False, ""])
    valid_bool = False
    reason = ""
    if isinstance(validation, list) and len(validation) >= 1:
        valid_bool = bool(validation[0])
    if isinstance(validation, list) and len(validation) >= 2:
        reason = str(validation[1])

    # Guardar en CSV
    csv_path = "validation_results.csv"
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
            writer.writerow(["prompt", "goal", "plan", "valid", "reason"])
        writer.writerow([prompt_text, goal, plan, valid_bool, reason])

    # Guardar en JSON (append)
    log_data = {
        "prompt": prompt_text,
        "goal": goal,
        "plan": plan,
        "validation": {"valid": valid_bool, "reason": reason},
        "model": state.get("model", "")
    }
    with open("final_log.json", "a", encoding='utf-8') as f:
        json.dump(log_data, f, ensure_ascii=False)
        f.write("\n")

    print("Logs finales guardados en validation_results.csv y final_log.json")
    return {"goal": goal, "plan": plan, "validation": [valid_bool, reason], "model": state.get("model", "")}



if __name__ == "__main__":

    graph_builder = StateGraph(State)
    graph_builder.add_node("chatgpt", chatgpt_chat)
    # graph_builder.add_node("gemini", gemini_chat)
    graph_builder.add_node("validate", validate_plan)
    graph_builder.add_node("save_logs", save_logs_node)
    graph_builder.set_entry_point("chatgpt")
    graph_builder.add_edge("chatgpt", "validate")
    graph_builder.add_edge("validate", "save_logs")
    graph_builder.add_edge("save_logs", END)
    graph = graph_builder.compile()
    graph.get_graph().draw_mermaid_png(output_file_path="graph.png")
    # El estado inicial debe ser un dict, no un string
    initial_state = {"goal": "", "plan": "", "validation": [False, ""]}
    graph.invoke(initial_state)
    # plan = """
    # 0.000: (start_welcome tiago)  [1.000]
    # 1.001: (move tiago home monalisa_ws)  [15.000]
    # 16.002: (explain_painting tiago monalisa_ws)  [15.000]
    # 31.002: (move tiago monalisa_ws dibejo_ws)  [15.000]
    # 46.003: (explain_painting tiago dibejo_ws)  [15.000]
    # 61.003: (move tiago dibejo_ws elgrito_ws)  [15.000]
    # 76.004: (explain_painting tiago elgrito_ws)  [15.000]
    # 91.005: (move tiago elgrito_ws guernica_ws)  [15.000]
    # 106.006: (explain_painting tiago guernica_ws)  [15.000]
    # 121.007: (move tiago guernica_ws nocheestrellada_ws)  [15.000]
    # 136.008: (recharge tiago nocheestrellada_ws)  [5.000]
    # 141.009: (move tiago nocheestrellada_ws home)  [15.000]
    # """
    # metrics_model(plan)

