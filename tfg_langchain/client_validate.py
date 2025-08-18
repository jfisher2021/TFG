import requests

def test_connection(server_url):
    try:
        r = requests.get(f"{server_url}/ping")
        print("🟢 Respuesta del servidor:", r.json())
    except Exception as e:
        print("🔴 Error de conexión:", e)

def test_validate(server_url, domain, problem, plan):
    with open(domain, "rb") as d, open(problem, "rb") as p, open(plan, "rb") as pl:
        files = {"domain": d, "problem": p, "plan": pl}
        data = {"timestep": "0.001"}
        r = requests.post(f"{server_url}/validate", files=files, data=data)
        print("✅ Resultado:")
        print(r.text)

if __name__ == \"__main__\":
    SERVER_URL = \"http://localhost:9000\"
    test_connection(SERVER_URL)
    test_validate(SERVER_URL, \"domain.pddl\", \"problem.pddl\", \"plan.txt\")
