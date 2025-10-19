"""Trim each file in log_gpt/: remove everything before the first line that starts with '0.000'.
Create a backup with extension .orig for each file before overwriting.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(__file__))
LOG_DIR = os.path.join(ROOT, 'log_gpt')

if __name__ == '__main__':
    if not os.path.isdir(LOG_DIR):
        print('No existe', LOG_DIR)
        raise SystemExit(1)

    files = [f for f in os.listdir(LOG_DIR) if os.path.isfile(os.path.join(LOG_DIR, f))]
    processed = 0
    skipped = 0

    for fname in files:
        path = os.path.join(LOG_DIR, fname)
        with open(path, 'r', encoding='utf-8') as f:
            text = f.read()

        # Buscar la primera línea que comienza con '0.000'
        m = re.search(r"^0\.000:\s*", text, flags=re.MULTILINE)
        if not m:
            skipped += 1
            continue

        start = m.start()
        new_text = text[start:]

        # Hacer copia de seguridad
        backup = path + '.orig'
        if not os.path.exists(backup):
            with open(backup, 'w', encoding='utf-8') as b:
                b.write(text)

        # Sobrescribir
        with open(path, 'w', encoding='utf-8') as out:
            out.write(new_text)

        print('Procesado:', fname)
        processed += 1

    print('Hecho. Procesados:', processed, 'Omitidos (sin 0.000):', skipped)
