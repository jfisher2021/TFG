"""Script para dividir log_gpt.txt en ficheros por plan.
Crea la carpeta `log_gpt/` en el repo y escribe ficheros `plan{n}_{modelo}.txt`.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(__file__))
LOG_PATH = os.path.join(ROOT, 'log_gpt.txt')
OUT_DIR = os.path.join(ROOT, 'log_gpt')

MODEL_HEADER_RE = re.compile(r"^\s*(\d+)\.\s*(.+)\s*$")  # ej: "1. GPT-5-instant"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    with open(LOG_PATH, 'r', encoding='utf-8') as f:
        text = f.read()

    # Normalizar: separar por bloques que empiezan con un número entero + ". " + modelo
    # Evitamos coincidir con timestamps como "0.000:" (tienen punto decimal seguido de más dígitos)
    header_re = re.compile(r"^\s*(\d+)\.\s+(.+)$", flags=re.MULTILINE)
    headers = []
    for m in header_re.finditer(text):
        headers.append((m.start(), int(m.group(1)), m.group(2).strip()))

    # Si no hay headers, escribir todo en un único fichero
    if not headers:
        out_file = os.path.join(OUT_DIR, 'plan1_unknown.txt')
        with open(out_file, 'w', encoding='utf-8') as out:
            out.write(text)
        print('No se detectaron cabeceras; creado', out_file)
        return

    # Extraer bloques: cada header hasta la siguiente header
    blocks = []
    for i, (s, num, model_raw) in enumerate(headers):
        start = s
        end = headers[i+1][0] if i+1 < len(headers) else len(text)
        block = text[start:end].strip('\n')
        blocks.append((num, model_raw, block))

    # Procesar cada bloque: extraer número y modelo de la primera línea
    for idx, (num, model_raw, block) in enumerate(blocks, start=1):
        # Limpiar model_raw: quitar URLs y contenido excesivo
        model_clean = re.sub(r"http\S+", "", model_raw)
        # Tomar hasta las dos primeras palabras alfanuméricas/hyphen
        m = re.match(r"^([A-Za-z0-9\-_.]+(?: [A-Za-z0-9\-_.]+)?)", model_clean)
        if m:
            modelo = m.group(1)
        else:
            modelo = model_clean.split()[0] if model_clean.split() else 'unknown'

        # Sanitizar para nombre de fichero: sólo alfanumérico, guión bajo o guión
        modelo = re.sub(r"[^A-Za-z0-9\-_]", "_", modelo)
        # Truncar para evitar nombres demasiado largos
        modelo = modelo[:40]

        filename = f"plan{num}_{modelo}.txt"
        out_path = os.path.join(OUT_DIR, filename)

        # Escribir (sobrescribe si existe)
        with open(out_path, 'w', encoding='utf-8') as out:
            out.write(block + '\n')

        print('Creado:', out_path)


if __name__ == '__main__':
    main()
