from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI
from google import genai
from google.genai import types
from dotenv import load_dotenv
import json
import sys
from langgraph.graph import END, StateGraph
from typing import Annotated, List, TypedDict, Union, Literal
import csv
from langchain.chat_models import init_chat_model
from pydantic import BaseModel, Field
from langchain_core.tools import tool


from langchain_ollama import ChatOllama
import ollama

sys.path.append(sys.path[0] + "/..")
from prompts import prompt_para_fichero_sin_goal, validate_plan_prompt
load_dotenv(override=True)

pddl_prompt = prompt_para_fichero_sin_goal
# Elegir el modelo según una condición


class State(TypedDict):
    goal: str
    plan: str
    # validation será una lista [bool, str]: [es_valido, razon]
    validation: List[object]

class InputGoal(TypedDict):
    goal: str
    # Estructuras de salida/validación más explícitas para el nodo de validación
class ResponseFormatter(BaseModel):
    """Always use this tool to structure your response to the user."""
    Format_Valid: Literal["S", "N"] = Field(description="The plan has a valid format (<S/N>)")
    Meets_Goal: Literal["S", "N"] = Field(description="The plan meets the goal (<S/N>) or not")
    Plan_Valid: Literal["S", "N"] = Field(description="The plan is valid (<S/N>) or not (correct format and meets the goal)")
    Goal: str = Field(description="The goal to be achieved")
    Errors: str = Field(description="Any errors found in the plan, if any")
    Comments: str = Field(description="Resume in 10 words the response")

@tool
def consult_csv() -> List[dict]:
    """Consult the CSV file for additional goal information."""
    print("LLAMANDO AL CSV")
    with open("cuadros.csv", "r", encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        # print([row for row in reader])
        return [row for row in reader]

def get_goal(goal: InputGoal) -> State:
    # model_with_csv_tool = init_chat_model("gpt-4o-mini", model_provider="openai").bind_tools([consult_csv])
    model_with_csv_tool = init_chat_model("gemini-2.5-pro", model_provider="google_genai").bind_tools([consult_csv])
    prompt_text = f"""You have to give the goal to the planner to create the plan. Your mission is to consult the CSV with the csv tool to get the plan.

    El objetivo dado es: {goal['goal']}"""

    # Primera invocación: el modelo puede devolver una propuesta y/o solicitar una tool (function_call)
    response = model_with_csv_tool.invoke(prompt_text)
    print("Respuesta inicial del modelo:", getattr(response, 'content', None), "additional_kwargs=", getattr(response, 'additional_kwargs', None))

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

            # Reenviar el resultado de la tool al modelo para que produzca la respuesta final
            follow_up = f"Tool {name} returned: {json.dumps(tool_result, ensure_ascii=False)}\nPlease now produce the final Goal in PDDL format only, taking into account the tool result and the original goal: {goal['goal']}"
            response2 = model_with_csv_tool.invoke(follow_up)
            print("Respuesta final tras ejecutar la tool:", getattr(response2, 'content', None))
            final_content = getattr(response2, 'content', '')
            return {"goal": final_content, "plan": "Hola", "validation": [False, ""]}

    # Si no hay llamadas a tools, usar el contenido devuelto (si existe)
    final = getattr(response, 'content', '') or ''
    return {"goal": final, "plan": "Hola", "validation": [False, ""]}

if __name__ == "__main__":

    graph_builder = StateGraph(State)       
    graph_builder.add_node("get_goal", get_goal)
    graph_builder.set_entry_point("get_goal")
    graph_builder.add_edge("get_goal", END)
    graph = graph_builder.compile()
    # graph.get_graph().draw_mermaid_png(output_file_path="graph.png")
    # El estado inicial debe ser un dict, no un string
    initial_state = {"goal": """visitar todos los cuadros españoles y franceses y Explicar Todos los cuadros Renacentistas y del Barroco""", 
    "plan": """adios""", "validation": [False, ""]}
    final_state = graph.invoke(initial_state)
    print("Flujo completado.")
    print("Estado final:", final_state)



