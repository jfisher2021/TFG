import sys

if __name__ == "__main__":
    entrada = sys.argv[1] if len(sys.argv) > 1 else "ningún mensaje"
    print(f"Hola desde el servidor. Recibí: {entrada}")
