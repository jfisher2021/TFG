import ollama 
# from ollama import chat
import sys
import os

client = ollama.Client()
# Listar todos los modelos disponibles
models = ollama.list()

# # Ver todos los modelos
# print("Modelos disponibles:\n")
# for idx, model in enumerate(models['models']):
#     print(f"{idx + 1}: {model['model']}")

# 0: my-assistant:latest
# 1: command-a:latest
# 2: deepseek-r1:70b
# 3: qwen3:235b
# 4: llama3.3:latest
# 5: qwen3:32b
# 6: deepseek-r1:32b
# 7: llama4:latest

# Solicitar al usuario seleccionar un modelo
# model_index = int(input("Selecciona el número del modelo que quieres usar: ")) - 1
model_index = 1
# Obtener el nombre del modelo seleccionado
selected_model = models['models'][model_index]['model']
# print(f"Has seleccionado el modelo: {selected_model}")
model = selected_model





# Cargar archivos PDDL
# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/domain.pddl', 'r') as f:
#     domain = f.read()

# with open('/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/pddl/problem.pddl', 'r') as f:
#     problem = f.read()

# Cargar CSV como texto plano
if len(sys.argv) > 1:
    user_input = sys.argv[1:]
    user_input = ' '.join(user_input)
    print(f"User input: {user_input}")
else:
    print("No input provided. Exiting.")
    sys.exit(1)

with open('/home/jfisher/tfg/cuadros.csv', 'r') as f:
    csv_data = f.read()



prompt = f"""
    Here is the CSV file content: 
    {csv_data}
    YOU are a museum guide. You have to explain the arts that i ask you about.
    You have to answer the questions about the CSV file.
    You can only use the information in the CSV file.
    Answer with no more than 200 words.
    Give a fun fact about the art if you think is important.
    Things you have to include in your answer:
    - Title of the art, Author of the art,  Year of creation, Style of the art ,Description of the art, 
    Context or historical significance, Fun fact (if available)

    IF some of the information is not available just dont say anything about it.
    An example of a question is: "What can you tell me about the art 'The Persistence of Memory' by Salvador Dalí?"

    OUTPUT FORMAT:
    The persistence of Memory by Salvador Dalí is a surrealist painting created in 1931. It is known for its dreamlike quality and melting clocks, symbolizing the fluidity of time. The painting is housed in the Museum of Modern Art in New York City. A fun fact about this piece is that it was inspired by a dream Dalí had after eating Camembert cheese.
    
    Da las respuestas en español y en texto plano, no en markdown, sin caracteres raros como * o ""
    You have to explain the art '{user_input}'
"""

response = client.generate(
    model=model,
    prompt=prompt,
    stream=True
    )

# print(response['response'])
for part in response:
    print(part['response'], end='', flush=True)


# while True:
#     # user_input = input('Chat with history: ')
#     
    
#     stream = chat(
#         model=model,
#         messages=[*messages, {'role': 'user', 'content': user_input}],
#         stream=True,
#     )

#     assistant_response = ""
#     for part in stream:
#         content = part['message']['content'] if 'message' in part and 'content' in part['message'] else part.get('content', '')
#         print(content, end='', flush=True)
#         assistant_response += content
#     print('\n')

#     # Add the response to the messages to maintain the history
#     messages += [
#         {'role': 'user', 'content': user_input},
#         {'role': 'assistant', 'content': assistant_response},
#     ]
#     break
