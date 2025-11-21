import os
import json
import sys
from typing import List, TypedDict
from pathlib import Path

import csv
from dotenv import load_dotenv
from groq import Groq
from google import genai
from langchain_openai import ChatOpenAI
from langgraph.graph import END, StateGraph
from langchain_core.tools import tool
from langchain.chat_models import init_chat_model
from openai import OpenAI


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import prompts

# Cargar CSV como texto plano
if len(sys.argv) > 1:
    user_input = ' '.join(sys.argv[1:])
    print(f"User input: {user_input}")
else:
    print("No input provided. Exiting.")
    sys.exit(1)

load_dotenv(override=True)
pddl_prompt = prompts.prompt_sin_ejemplos_input_goal
STREAMING = True




class State(TypedDict):
    state_goal: str
    state_plan: str
    state_prompt: str

@tool
def consult_csv() -> List[dict]:
    """Consulta el archivo CSV con información detallada de todos los cuadros disponibles en el museo (nombre, autor, estilo, país de origen, etc.)."""
    print("CONSULTANDO CSV CON INFORMACIÓN DE CUADROS...")
    with open(ROOT_DIR / "cuadros.csv", "r", encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        return [row for row in reader]

def get_goal(state: State) -> State:
    # model_with_csv_tool = init_chat_model("gpt-4o-mini", model_provider="openai").bind_tools([consult_csv])
    model_with_csv_tool = init_chat_model("openai/gpt-oss-120b", model_provider="groq").bind_tools([consult_csv])
    
    prompt_text = prompts.prompt_get_goal_con_csv.format(goal=state['state_goal'])

   
    response = model_with_csv_tool.invoke(prompt_text)
    print("Respuesta inicial del modelo:", getattr(response, 'content', None))

    # Si el modelo solicitó una llamada a la herramienta, ejecutar la tool manualmente y reenviarla al modelo
    tool_calls = getattr(response, 'tool_calls', None) or (response.additional_kwargs.get('state_function_call') if getattr(response, 'additional_kwargs', None) else None)
    if tool_calls:
        # soportar lista de tool_calls o un único dict
        calls = tool_calls if isinstance(tool_calls, list) else [tool_calls]
        for call in calls:
            name = call.get('name') if isinstance(call, dict) else None
            print(f"Modelo solicitó la tool: {name}")
            # Por ahora manejamos solo la tool consult_csv sin argumentos
            if name == 'consult_csv':
                try:
                    # Las tools de langchain-core exponen `invoke` para ejecución programática
                    tool_result = consult_csv.invoke({})
                except Exception as e:
                    print("Error al invocar consult_csv.invoke:", e)
                    tool_result = None
            else:
                # Si hay otras tools, no manejadas explícitamente, omitir
                tool_result = None

            follow_up = f"""Resultado de la herramienta {name}:
                {json.dumps(tool_result, ensure_ascii=False, indent=2)}

                Ahora genera el objetivo final en formato PDDL considerando:
                - La información del CSV proporcionada arriba
                - El objetivo original del usuario: {state['state_goal']}

                Responde ÚNICAMENTE con el objetivo en formato PDDL, usando unicamente  los predicados (visited ...) y (explained_painting ...)"""
            final_content = ""
            if STREAMING:
                print("Iniciando respuesta en streaming tras ejecutar la tool...")
                for chunk in model_with_csv_tool.stream(follow_up):
                    content = getattr(chunk, 'content', '')
                    if content:
                        print(content, end='', flush=True)
                        final_content += content
                print("\n")
                return {"state_goal": final_content, "state_plan": "", "state_prompt": pddl_prompt.format(GOAL=final_content)}
            else:        
                response2 = model_with_csv_tool.invoke(follow_up)
                print("Respuesta final tras ejecutar la tool:", getattr(response2, 'content', None))
                final_content = getattr(response2, 'content', '')
                return {"state_goal": final_content, "state_plan": "", "state_prompt": pddl_prompt.format(GOAL=final_content)}

    # Si no hay llamadas a tools, usar el contenido devuelto (si existe)
    print("No hace falta llamar a ninguna tool")
    final = getattr(response, 'content', '') or ''
    return {"state_goal": final, "state_plan": "", "state_prompt": pddl_prompt.format(GOAL=final)}

def generate_plan(state: State):
    """Genera un plan PDDL usando el modelo especificado."""
    # Configurar el modelo a usar (puedes cambiar esta variable)
    MODEL_TO_USE = "groq"  # Opciones: "gemini", "chatgpt", "groq"
    
    full_response = ""
    
    if MODEL_TO_USE == "gemini":
        print("USANDO GEMINI-2.5-flash") 
        selected_model = "gemini-2.5-flash"
        client = genai.Client()
        response = client.models.generate_content(
            model=selected_model, contents=state.get("state_prompt"),
            config=genai.types.GenerateContentConfig(
                temperature=0.0,
            ),
        )
        print("\nGenerando plan PDDL...")
        print("=" * 50)
        print(response.text)
        full_response = response.text
        
    elif MODEL_TO_USE == "chatgpt":
        print("USANDO GPT-4O-MINI")
        selected_model = "gpt-4o-mini"
        model = ChatOpenAI(
            model=selected_model,
            temperature=1.0,
        )
        print(state.get("state_prompt"))
        for chunk in model.stream(state.get("state_prompt")):
            print(chunk.content, end='', flush=True)
            full_response += chunk.content
        print("\nGenerando plan PDDL...")
        print("=" * 50)
        
    elif MODEL_TO_USE == "groq":
        print("USANDO GROQ")
        selected_model = "openai/gpt-oss-120b"
        api_key = os.getenv("GROQ_API_KEY")
        base_url = "https://api.groq.com/openai/v1"
        client = OpenAI(base_url=base_url, api_key=api_key)

        response = client.chat.completions.create(
            model=selected_model,
            messages=[
                {
                    "role": "user",
                    "content": state.get("state_prompt")
                }
            ],
            # temperature=0.2,
            stream=STREAMING
        )
        if STREAMING:
            for sse_chunk in response:
                content = sse_chunk.choices[0].delta.content
                if content:
                    print(content, end="")
                    full_response += content
        else:
            full_response = response.choices[0].message.content
            print(full_response)
        print("\nGenerando plan PDDL...")
        print("=" * 50)
    
    # print_and_save_logs(selected_model, state.get("state_prompt"), full_response)
    return {"state_goal": state.get("state_goal", ""), "state_plan": full_response, "state_prompt": state.get("state_prompt", "")}


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

if __name__ == "__main__":

    # Crear el grafo
    graph_builder = StateGraph(State)

    # Añadir los nodos
    graph_builder.add_node("get_goal", get_goal)
    graph_builder.add_node("generate_plan", generate_plan)

    # Conectar los nodos
    graph_builder.set_entry_point("get_goal")
    graph_builder.add_edge("get_goal", "generate_plan")
    graph_builder.add_edge("generate_plan", END)

    # Compilar el grafo
    graph = graph_builder.compile()
    # graph.get_graph().draw_mermaid_png(output_file_path="final_graph.png")
    # print(graph.get_graph().draw_mermaid())
    initial_state = {"state_goal": user_input, "state_plan": "", "state_prompt": pddl_prompt}
    final_state = graph.invoke(initial_state)
    