TFG

## tfg-ia como subtree

Este repositorio incluye el código de `tfg-ia` mediante Git Subtree bajo el directorio `tfg-ia/`.

- Remoto configurado: `tfg-ia-remote` → https://github.com/jfisher2021/tfg-ia.git
- Rama seguida: `main`

### Clonado normal

No requiere pasos adicionales: al clonar este repo se obtiene también el contenido de `tfg-ia/`.

### Actualizar `tfg-ia` desde su repo original

Puedes usar el script de ayuda:

```
scripts/update_tfg_ia.sh
```

O manualmente:

1. Traer cambios del remoto del subtree:
	- `git fetch tfg-ia-remote main`
2. Incorporar cambios al directorio `tfg-ia/`:
	- `git subtree pull --prefix=tfg-ia tfg-ia-remote main -m "chore(subtree): sync tfg-ia"`

### Enviar cambios hechos aquí hacia `tfg-ia`

Si haces cambios dentro de `tfg-ia/` y deseas empujarlos al repo `tfg-ia`:

```
git subtree push --prefix=tfg-ia tfg-ia-remote main
```

### Notas

- Durante la migración se dejó una copia temporal `tfg-ia.tmp/` (ignorad@ en `.gitignore`) por si hubiera archivos locales no versionados (por ejemplo `.env`, `.venv`, `__pycache__`, logs). Revísala y elimina lo que no necesites.
- No se versionan archivos sensibles (`.env`) ni artefactos (`.venv`, `__pycache__`).

