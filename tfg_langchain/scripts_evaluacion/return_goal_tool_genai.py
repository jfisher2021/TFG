import csv
import json
import sys
from pathlib import Path
from typing import List, TypedDict

from dotenv import load_dotenv
from google import genai
from google.genai import types

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from prompts import prompt_get_goal_con_csv

load_dotenv(override=True)


MODEL = "gemini-2.5-flash"

class State(TypedDict):
    goal: str
    plan: str
    validation: List[object]


class InputGoal(TypedDict):
    goal: str


def consult_csv() -> List[dict]:
    """Consulta el archivo CSV para obtener información sobre los cuadros disponibles."""
    print(" CONSULTANDO CSV...")
    with open(ROOT_DIR / "cuadros.csv", "r", encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        return [row for row in reader]


def get_goal(goal_text: str) -> State:
    """
    Procesa el objetivo del usuario usando Gemini.
    Solo llama a la tool consult_csv si el modelo lo solicita explícitamente.
    """
    client = genai.Client()
    
    # Definir la tool de consulta CSV para Gemini
    csv_tool = types.Tool(
        function_declarations=[
            types.FunctionDeclaration(
                name="consult_csv",
                description="Consulta el archivo CSV para obtener información detallada sobre los cuadros disponibles en el museo (nombre, autor, estilo, país, etc.)",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={}
                )
            )
        ]
    )
    
    prompt_text = prompt_get_goal_con_csv.format(goal=goal_text)

    print(" Llamando a Gemini...")
    
    # Primera llamada con la tool disponible
    response = client.models.generate_content(
        model=MODEL,
        contents=prompt_text,
        config=types.GenerateContentConfig(
            tools=[csv_tool],
            temperature=0.1
        )
    )
    # Verificar si el modelo quiere llamar a la tool
    if response.candidates[0].content.parts:
        for part in response.candidates[0].content.parts:
            # Si hay una function_call, ejecutar la tool
            if hasattr(part, 'function_call') and part.function_call:
                function_name = part.function_call.name
                print(f" Modelo solicitó la tool: {function_name}")
                
                if function_name == "consult_csv":
                    # Ejecutar la tool
                    csv_data = consult_csv()
                    
                    # Segunda llamada con el resultado de la tool
                    print(" Enviando resultado de la tool a Gemini...")
                    
                    response = client.models.generate_content(
                        model=MODEL,
                        contents=[
                            types.Content(
                                role="user",
                                parts=[types.Part(text=prompt_text)]
                            ),
                            types.Content(
                                role="model",
                                parts=[part]  # La function_call original
                            ),
                            types.Content(
                                role="function",
                                parts=[
                                    types.Part(
                                        function_response=types.FunctionResponse(
                                            name=function_name,
                                            response={"result": csv_data}
                                        )
                                    )
                                ]
                            )
                        ],
                        config=types.GenerateContentConfig(
                            temperature=0.1
                        )
                    )
    else: 
        print(" No se requiere información adicional del CSV.")
    
    # Extraer el texto final de la respuesta
    final_goal = response.text if hasattr(response, 'text') else ""
    
    print(f"\n Goal final generado:\n{final_goal}\n")
    
    return {
        "goal": final_goal,
        "plan": "",
        "validation": [False, ""]
    }


if __name__ == "__main__":
    # Prueba directa sin LangGraph
    # user_goal = """visitar todos los cuadros españoles y franceses y Explicar Todos los cuadros Renacentistas y del Barroco"""
    # user_goal = "Mira en el csv y explica todos los cuadros de picasso "
    user_goal = "(visit Guernica) no hace falta ver el csv "
    print("="*60)
    print("INICIANDO PROCESAMIENTO DE GOAL")
    print("="*60)
    print(f"Goal del usuario: {user_goal}\n")
    
    result = get_goal(user_goal)
    
    print("="*60)
    print("RESULTADO FINAL")
    print("="*60)
    print(f"Goal: {result['goal']}")
    print(f"Plan: {result['plan']}")
    print(f"Validation: {result['validation']}")



