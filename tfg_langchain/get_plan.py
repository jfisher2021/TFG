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
    model_with_csv_tool = init_chat_model("gemini-2.5-flash", model_provider="google_genai").bind_tools([consult_csv])
    
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
                        
            response2 = model_with_csv_tool.invoke(follow_up)
            print("Respuesta final tras ejecutar la tool:", getattr(response2, 'content', None))
            final_content = getattr(response2, 'content', '')
            return {"state_goal": final_content, "state_plan": "", "state_prompt": pddl_prompt.format(GOAL=final_content)}

    # Si no hay llamadas a tools, usar el contenido devuelto (si existe)
    print("No hace falta llamar a ninguna tool")
    final = getattr(response, 'content', '') or ''
    return {"state_goal": final, "state_plan": "", "state_prompt": pddl_prompt.format(GOAL=final)}

def gemini_chat(state: State):
    print("USANDO GEMINI-2.5-PRO") 
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
    print_and_save_logs(selected_model, state.get("state_prompt"), full_response)
    # Inicializar validation como [False, ""]; la validación real la hará el nodo validate
    return {"state_goal": state.get("state_goal", ""), "state_plan": full_response, "state_prompt": state.get("state_prompt", "")}

def chatgpt_chat(state: State):
    # Crear el modelo de LangChain con OpenAI
    print("USANDO GPT-4O-MINI")
    selected_model = "gpt-4o-mini"
    model = ChatOpenAI(
        model=selected_model,
        temperature=1.0,
    )
    full_response = ""
    print(state.get("state_prompt"))
    for chunk in model.stream(state.get("state_prompt")):
        print(chunk.content, end='', flush=True)
        full_response += chunk.content

    print("\nGenerando plan PDDL...")
    print("=" * 50)
    print_and_save_logs(selected_model, state.get("state_prompt"), full_response)
    return {"state_goal": state.get("state_goal", ""), "state_plan": full_response, "state_prompt": state.get("state_prompt", "")}

def groq_chat(state: State):
    # Crear el modelo de LangChain con OpenAI
    print("USANDO GROQ")
    client = Groq()
    completion = client.chat.completions.create(
        model="openai/gpt-oss-20b",        
        messages=[
            {
                "role": "user",
                "content": state.get("state_prompt")
            }
        ],
        temperature=0.3,
        max_completion_tokens=8192,
        top_p=1,
        reasoning_effort="medium",
        stop=None
    )

    full_response = completion.choices[0].message.content
    selected_model = "openai/gpt-oss-20b"
    
    print(full_response)
    print("\nGenerando plan PDDL...")
    print("=" * 50)
    print_and_save_logs(selected_model, state.get("state_prompt"), full_response)
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
    # graph_builder.add_node("chatgpt", chatgpt_chat)
    # graph_builder.add_node("groq", groq_chat)
    graph_builder.add_node("gemini", gemini_chat)

    # Conectar los nodos
    graph_builder.set_entry_point("get_goal")
    graph_builder.add_edge("get_goal", "gemini")
    graph_builder.add_edge("gemini", END)

    # Compilar el grafo
    graph = graph_builder.compile()

    # graph.get_graph().draw_mermaid_png(output_file_path="graph.png")
    # El estado inicial debe ser un dict, no un string
    initial_state = {"state_goal": user_input, "state_plan": "", "state_prompt": pddl_prompt}

    final_state = graph.invoke(initial_state)

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

