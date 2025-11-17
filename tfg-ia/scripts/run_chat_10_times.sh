#!/bin/bash
# Script para ejecutar chat_flujo_completo.py 10 veces de forma secuencial
# Cada ejecución espera a que termine la anterior

# Determinar ruta relativa a la carpeta del script (proyecto tfg-ia)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)" 

PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"
SCRIPT_PATH="$PROJECT_ROOT/tfg_langchain/scripts_evaluacion/chat_flujo_completo.py"
LOG_FILE="$PROJECT_ROOT/logs_script.txt"


# Redirigir toda la salida al fichero de logs
{
    echo "Iniciando ejecución secuencial de chat_flujo_completo.py (10 veces)"
    echo "=================================================================="
    echo "Fecha de inicio: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""

    for i in {1..10}
    do
        echo ""
        echo ">>> Ejecución #$i de 10 - $(date '+%Y-%m-%d %H:%M:%S')"
        echo "-------------------------------------------------------------------"
        
        # Ejecutar el script y esperar a que termine
        $PYTHON_BIN $SCRIPT_PATH 2>&1
        
        # Capturar el código de salida
        EXIT_CODE=$?
        
        if [ $EXIT_CODE -eq 0 ]; then
            echo "✓ Ejecución #$i completada exitosamente"
        else
            echo "✗ Ejecución #$i falló con código de salida: $EXIT_CODE"
            echo "Continuando con las siguientes ejecuciones..."
        fi
        
        # Pequeña pausa entre ejecuciones
        if [ $i -lt 10 ]; then
            echo "Esperando 2 segundos antes de la siguiente ejecución..."
            sleep 2
        fi
    done

    echo ""
    echo "=================================================================="
    echo "Completadas todas las ejecuciones - $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=================================================================="
} >> "$LOG_FILE" 2>&1

echo "Ejecución completada. Logs guardados en: $LOG_FILE"
