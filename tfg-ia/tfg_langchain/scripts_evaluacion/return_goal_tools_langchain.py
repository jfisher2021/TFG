import csv
import json
import sys
from pathlib import Path
from typing import List, TypedDict, Literal

from dotenv import load_dotenv
from langchain.chat_models import init_chat_model
from langchain_core.tools import tool
from langgraph.graph import END, StateGraph
from pydantic import BaseModel, Field


ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import prompt_get_goal_con_csv, prompt_sin_ejemplos_input_goal, validate_plan_prompt
load_dotenv(override=True)

pddl_prompt = prompt_sin_ejemplos_input_goal
# Elegir el modelo según una condición

class State(TypedDict):
    goal: str
    plan: str
    validation: List[object]

class InputGoal(TypedDict):
    goal: str
    # Estructuras de salida/validación más explícitas para el nodo de validación

@tool
def consult_csv() -> List[dict]:
    """Consulta el archivo CSV con información detallada de todos los cuadros disponibles en el museo (nombre, autor, estilo, país de origen, etc.)."""
    print("CONSULTANDO CSV CON INFORMACIÓN DE CUADROS...")
    with open(ROOT_DIR / "cuadros.csv", "r", encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        return [row for row in reader]

def get_goal(goal: InputGoal) -> State:
    # model_with_csv_tool = init_chat_model("gpt-4o-mini", model_provider="openai").bind_tools([consult_csv])
    model_with_csv_tool = init_chat_model("gemini-2.5-flash", model_provider="google_genai").bind_tools([consult_csv])
    
    prompt_text = prompt_get_goal_con_csv.format(goal=goal['goal'])

   
    response = model_with_csv_tool.invoke(prompt_text)
    print("Respuesta inicial del modelo:", getattr(response, 'content', None))

    # Si el modelo solicitó una llamada a la herramienta, ejecutar la tool manualmente y reenviarla al modelo
    tool_calls = getattr(response, 'tool_calls', None) or (response.additional_kwargs.get('function_call') if getattr(response, 'additional_kwargs', None) else None)
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
                - El objetivo original del usuario: {goal['goal']}

                Responde ÚNICAMENTE con el objetivo en formato PDDL, usando unicamente  los predicados (visited ...) y (explained_painting ...)"""
                        
            response2 = model_with_csv_tool.invoke(follow_up)
            print("Respuesta final tras ejecutar la tool:", getattr(response2, 'content', None))
            final_content = getattr(response2, 'content', '')
            return {"goal": final_content, "plan": "", "validation": [False, ""]}

    # Si no hay llamadas a tools, usar el contenido devuelto (si existe)
    print("No hace falta llamar a ninguna tool")
    final = getattr(response, 'content', '') or ''
    return {"goal": final, "plan": "", "validation": [False, ""]}

if __name__ == "__main__":
    graph_builder = StateGraph(State)       
    graph_builder.add_node("get_goal", get_goal)
    graph_builder.set_entry_point("get_goal")
    graph_builder.add_edge("get_goal", END)
    graph = graph_builder.compile()
    # graph.get_graph().draw_mermaid_png(output_file_path="graph.png")
    # El estado inicial debe ser un dict, no un string
    initial_state = {"goal": """quiero ver todos las meninas y los de picasso""", 
    "plan": """adios""", "validation": [False, ""]}
    # initial_state = {"goal": """(:goal (and\n    (visitado Guernica)\n    (visitado Las-Meninas)\n    (visitado El-3-de-mayo-de-1808)\n    (visitado La-rendicion-de-Breda)\n    (visitado La-persistencia-de-la-memoria)\n    (visitado Saturno-devorando-a-su-hijo)\n    (visitado El-carnaval-del-arlequin)\n    (visitado Maestro-Aprendiz)\n    (visitado La-libertad-guiando-al-pueblo)\n    (visitado Impresion-sol-naciente)\n    (visitado Banistas-en-Asnieres)\n    (explicado Mona-Lisa)\n    (explicado El-jardin-de-las-delicias)\n    (explicado El-nacimiento-de-Venus)\n    (explicado La-creacion-de-Adan)\n    (explicado La-ultima-cena)\n    (explicado La-joven-de-la-perla)\n    (explicado Las-Meninas)\n    (explicado Las-tres-gracias)\n    (explicado La-rendicion-de-Breda)\n    (explicado La-ronda-de-noche)\n))\n""", 
    # "plan": """adios""", "validation": [False, ""]}
    final_state = graph.invoke(initial_state)
    print("\n" + "="*60)
    print("FLUJO COMPLETADO")
    print("="*60)
    print("Estado final:", final_state)



