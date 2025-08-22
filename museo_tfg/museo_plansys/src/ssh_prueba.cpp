#include <cstdio>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Uso: " << argv[0] << " <argumento_para_python>\n";
    return 1;
  }
  // std::string python_arg = argv[1];
  std::string python_arg =  "Dame 5 cuadros aleatorios";
  std::string command = "ssh dedalo.tsc.urjc.es 'python3 /home/jfisher/tfg/tfg_ollama/probando_cuadros.py " + python_arg + "'";
  std::string result;
  // system("ls"); // Clear the terminal screen for better visibility
  std::cout << "Ejecutando comando remoto: " << command << std::endl;

  char buffer[128];

  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    std::cerr << "Error abriendo el pipe\n";
    return 1;
  }

  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }

  pclose(pipe);
  std::cout << "Salida remota:\n" << result << std::endl;
  return 0;
}
