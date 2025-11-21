import json
import sys
from pathlib import Path

import ollama


def write_log_txt(file_name, model, prompt, response):
    """Escribe logs en formato texto en ollama_planner/logs/"""
    # Crear directorio logs en ollama_planner si no existe
    ollama_planner_dir = Path(__file__).resolve().parent / "ollama_planner"
    logs_dir = ollama_planner_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    log_file = logs_dir / file_name
    with open(log_file, "a", encoding="utf-8") as f:
        f.writelines(["\nMODEL : ", model, "\nPROMPT:\n", prompt, "\nRESPONSE:\n", response["response"]])
    # print(f"LOG: guardado en: {log_file}")


def write_log_json(file_name, model, prompt, response):
    """Escribe logs en formato JSON en ollama_planner/logs/"""
    # Crear directorio logs en ollama_planner si no existe
    ollama_planner_dir = Path(__file__).resolve().parent / "ollama_planner"
    logs_dir = ollama_planner_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    # Datos que deseas guardar
    log_data = {
        "model": model,
        "prompt": prompt,
        "response": response["response"]
    }
    
    log_file = logs_dir / file_name
    with open(log_file, "a", encoding="utf-8") as file:
        json.dump(log_data, file, ensure_ascii=False)
        file.write("\n")  # Nueva línea para separar cada registro
    # print(f"LOG: guardado en: {log_file}")


def select_model_ollama() -> str:
    """Lista modelos de Ollama disponibles y permite al usuario seleccionar uno"""
    # Listar todos los modelos disponibles
    models = ollama.list()

    # Ver todos los modelos
    print("Modelos disponibles:\n")
    for idx, model in enumerate(models['models']):
        print(f"{idx + 1}: {model['model']}")

    # Solicitar al usuario seleccionar un modelo
    model_index = int(input("Selecciona el número del modelo que quieres usar: ")) - 1

    # Obtener el nombre del modelo seleccionado
    selected_model = models['models'][model_index]['model']
    print(f"Has seleccionado el modelo: {selected_model}")
    return selected_model