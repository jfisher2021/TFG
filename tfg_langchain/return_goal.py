from langchain.chat_models import init_chat_model
from dotenv import load_dotenv
import sys
import os

load_dotenv(override=True)
model = init_chat_model(model="gemini-2.5-pro", model_provider="google-genai" )



# Cargar CSV como texto plano
if len(sys.argv) > 1:
    user_input = sys.argv[1:]
    user_input = ' '.join(user_input)
    print(f"User input: {user_input}")
else:
    print("No input provided. Exiting.")
    sys.exit(1)

with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/tfg-ia/cuadros.csv', 'r') as f:
    csv_data = f.read()



prompt = f"""
    Here is the CSV file content: 
    {csv_data}
    You have to give the goal to the planner to create the plan. Your mission is to consult the CSV to get the plan.
    You can only do 2 actions
    (visited tiago DRAWING)
    (explained_painting DRAWING)
     '{user_input}'
"""

response = model.invoke(prompt)

print(response.content)