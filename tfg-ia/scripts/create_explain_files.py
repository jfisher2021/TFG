### Script para crear archivos de explicación para cada cuadro en cuadros.csv

from langchain.prompts.prompt import PromptTemplate
from google import genai
from google.genai import types
from dotenv import load_dotenv
from pathlib import Path
import sys
import csv
import asyncio

# asegurar que el root del repo está en sys.path para importar prompts
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from prompts import explain_drawings
except Exception:
    # Fallback: leer prompts.py y extraer la variable explain_drawings sin ejecutar el módulo
    PROMPTS_PATH = ROOT / "prompts.py"
    if not PROMPTS_PATH.exists():
        raise
    text = PROMPTS_PATH.read_text(encoding="utf-8")
    # buscar la asignación explain_drawings = f"""..."""
    m = __import__("re").search(r"explain_drawings\s*=\s*f?\"\"\"(.*?)\"\"\"", text, __import__("re").DOTALL)
    if not m:
        raise ImportError("No se pudo extraer 'explain_drawings' desde prompts.py")
    explain_drawings = m.group(1)
import argparse
from pathlib import Path

load_dotenv()


def build_prompt(csv_text: str, art_name: str) -> str:
    tpl = PromptTemplate(input_variables=["csv_data", "user_input"], template=explain_drawings)
    return tpl.format(csv_data=csv_text, user_input=art_name)


async def call_gemini_async(client: genai.Client, prompt_text: str, model: str = "gemini-2.5-pro", temperature: float = 0.7):
    """Llamada asíncrona usando client.aio.models.generate_content."""
    print(f"USANDO MODELO: {model}")
    resp = await client.aio.models.generate_content(
        model=model,
        contents=prompt_text,
        config=genai.types.GenerateContentConfig(temperature=temperature),
    )
    return resp.text


async def _process_drawing(client: genai.Client, drawing: str, csv_text: str, model: str, sem: asyncio.Semaphore):
    safe_name = __import__("re").sub(r'[^A-Za-z0-9_.-]+', '_', drawing) if drawing else "unknown"
    # escribir respuestas como archivos planos en explicacion_respuestas/
    out_dir = ROOT / "explicacion_respuestas"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file_path = out_dir / f"{safe_name}.txt"
    try:
        async with sem:
            prompt_text = build_prompt(csv_text, drawing)
            out = await call_gemini_async(client, prompt_text, model=model)
        # escribir resultado sin bloquear el loop
        await asyncio.to_thread(lambda p, t: p.write_text(t, encoding="utf-8"), out_file_path, out)
        print(f"Respuesta guardada en: {out_file_path}")
    except Exception as e:
        print(f"Error al procesar {drawing}:", e)
        # escribir error en archivo separado
        await asyncio.to_thread(lambda p, t: p.write_text(t, encoding="utf-8"), out_dir / f"{safe_name}_error.txt", str(e))


def _cleanup_old_explicacion_dirs(root: Path):
    """Mover archivos respuesta.txt de carpetas antiguas en root/explicacion/* a explicacion_respuestas/
    y eliminar las carpetas antiguas.
    """
    old_base = root / "explicacion"
    new_base = root / "explicacion_respuestas"
    if not old_base.exists():
        return
    new_base.mkdir(parents=True, exist_ok=True)
    for child in old_base.iterdir():
        if child.is_dir():
            resp = child / "respuesta.txt"
            err = child / "error.txt"
            safe_name = child.name
            if resp.exists():
                try:
                    dest = new_base / f"{safe_name}.txt"
                    dest.write_text(resp.read_text(encoding="utf-8"), encoding="utf-8")
                except Exception:
                    pass
            if err.exists():
                try:
                    dest_err = new_base / f"{safe_name}_error.txt"
                    dest_err.write_text(err.read_text(encoding="utf-8"), encoding="utf-8")
                except Exception:
                    pass
            # intentar eliminar la carpeta
            try:
                for f in child.iterdir():
                    try:
                        f.unlink()
                    except Exception:
                        pass
                child.rmdir()
            except Exception:
                pass


async def main():
    csv_path = Path(ROOT / "cuadros.csv")
    if not csv_path.exists():
        print(f"CSV no encontrado: {csv_path}")
        return 2

    model = "gemini-2.5-flash"
    concurrency = 4
    sem = asyncio.Semaphore(concurrency)

    # leer CSV y texto completo
    csv_text = await asyncio.to_thread(csv_path.read_text, "utf-8")

    def _read_rows(path):
        res = []
        with open(path, newline='') as File:
            reader = csv.reader(File)
            next(reader, None)
            for row in reader:
                if row:
                    val = row[0].strip()
                    if val:
                        res.append(val)
        return res

    drawings = await asyncio.to_thread(_read_rows, csv_path)
    if not drawings:
        print(f"No se encontró ningún cuadro en: {csv_path}")
        return 2

    client = genai.Client()

    tasks = [asyncio.create_task(_process_drawing(client, d, csv_text, model, sem)) for d in drawings]
    await asyncio.gather(*tasks)
    return 0


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    raise SystemExit(exit_code)