[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/dMkD-Mt7)
# Trabajo Final Planificación - La Biblioteca

El trabajo final consiste en implementar en equipo un sistema de planificación para un robot que trabaje en una biblioteca.
El robot será capaz de realizar una serie de tareas (misiones) que pueden estar compuestas por una o más acciones. Como mínimo el robot debe poder ejecutar 3 tareas distintas.
A continuación se muestra un ejempo de posibles tareas a ejecutar:

* Atender visitas
* Recoger los libros de las mesas
* Resolver el cubo de Rubik
* Buscar un libro en las estanterías
* Recoger la basura
* Mandar callar a las personas ruidosas

Se dispone de libertad para elegir las tareas y para definir el comportamiento del robot, con las siguientes condciones:

* Se deben definir varios "waypoints" en la bilioteca (entrada, estanterías, mesas, etc.) para las distintas tareas.
* Al menos una de las tareas deberá inlcuir una componente aleatoria. Por ejemplo:
    * Que a la hora de buscar un libro, el robot vaya recorriendo estanterías hasta que lo encuentre, y que se pueda dar el caso en el que no exista el libro.
    * Que la acción de atender las visitas implique acercarse a la puerta a ver si hay alguien. Si hay una persona, se acompaña a esa persona a un punto de la biblioteca. Si no hay ninguna persona, la misión termina. Que haya una persona o no se puede decidir de forma aleatoria en el instante de "detectar" a la persona.



## Ejercicio 1 - Mundo simulado

### Mundo
Se utilizará un mundo simulado en gazebo de una biblioteca. Podéis encontrar un mundo funcional en [este repositorio](https://github.com/Juancams/aws-robomaker-bookstore-world/tree/ros2):

![world](assets/sim_world.png)

### Robot
Se puede utilizar el modelo de cualquier robot, aunque se recomiendan los simuladores del [Kobuki](https://github.com/IntelligentRoboticsLabs/kobuki) o del [TIAGo](https://github.com/Tiago-Harmonic/tiago_harmonic), que ya se han utilizado en otras asignaturas.

Al utilizar un robot de verdad, las tareas de navegación deberán utilizar nav2 para que el robot se mueva de verdad. El resto de las acciones pueden seguir siendo sintéticas. Es decir, no es necesario que el robot interactúe con los objetos de la biblioteca.

Indicar qué simulador se ha utilizado, cómo ejecutarlo y cómo lanzar el sistema de planificación para que actúe sobre el robot:

## Respuesta

Para poder lanzar el simulador y Nav2 hay que descargar varios paquetes.

### Instalación de paquetes necesarios

### 1. Kobuki


```bash
source /opt/ros/<ROS-DISTRO>/setup.bash
```

Clone the repository to your workspace:
```bash
cd <ros2-workspace>/src
git clone https://github.com/IntelligentRoboticsLabs/kobuki.git -b rolling
```

Prepare your thirparty repos:
```bash
sudo apt update && sudo apt install ros-dev-tools -y
cd <ros2-workspace>/src/
vcs import < kobuki/thirdparty.repos
```
*Please make sure that this last command has not failed. If this happens, run it again.*

### Install libusb, libftdi & libuvc
```bash
sudo apt install libusb-1.0-0-dev libftdi1-dev libuvc-dev
```

### Install udev rules from astra camera, kobuki and rplidar
When you connect a piece of hardware to your pc, it assigns `/dev/ttyUSB*` to it. This will not have the necessary read/write permissions, so we will not be able to use it correctly. The solution is to set up some udev rules that creates a symlink with another name (example: `/dev/ttyUSB0` -> `/dev/kobuki`) and grants it the necessary permissions.
```bash
cd <ros2-workspace>
sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/kobuki_ros/60-kobuki.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Building project
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install 
```

>  If your terminal has crashed or closed while compiling, please try compiling your packages as follows `colcon build --symlink-install --parallel-workers 1` or do so by selecting the package that failed `colcon build --symlink-install --parallel-workers 1 --packages-select <package>`
> 
> Also, if you want to prevent it from recompiling that package, add a `COLCON_IGNORE` inside the package
 
### Run Navigation in ROS 2

You can use [Nav2](https://navigation.ros.org/) using robot with this launcher:

```bash
ros2 launch kobuki navigation.launch.py map:=<path-to-map>
``` 

or this other command if you need to navigate in the simulator
```bash
ros2 launch kobuki navigation_sim.launch.py
```

If you want to use another map, you have to put the route in the map parameter

### 2. NAV2 

Si tienes instalado Nav2 pero te da error al lanzar el nodo de navegación, ejecuta el siguiente comando:

```bash
cd <ros2-workspace>/src/nav2
git checkout  fbd1d3e7964dc220c0861d5e8bdcfcc7b9ec2812
```

### 3. Common_interfaces

Al igual que con Nav2, si tienes problemas con el paquete `common_interfaces`, ejecuta el siguiente comando:

```bash
cd <ros2-workspace>/src/common_interfaces
git checkout 0cf96ab111a3da980a1f94cf18d6867701d70939
```

### EJECUCIÓN

Para lanzar el simulador y el sistema de planificación, se deben ejecutar los siguientes comandos:

Gazebo:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch kobuki simulation.launch.py world:=install/aws_robomaker_bookstore_world/share/aws_robomaker_bookstore_world/worlds/bookstore.world 
```

Nav2:

```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch kobuki navigation_sim.launch.py map:=src/kobuki/maps/aws_bookstore.yaml 
```


## Ejercicio 2 - Dominio PDDL

Hay que diseñar e implementar un dominio en PDDL desde cero, donde se modele el mundo en el que va a operar el robot y las distintas acciones que este puede ejecutar. A la hora de diseñar las acciones, es importante pensar en cómo van a estar implementadas (Ejercicio 3).

Indicad la lista de acciones implementadas y la lista de tipos, predicados y fluents necesarios para vuestro modelo:

**Respuesta**

Para crear un plan, se utiliza un [dominio](trabajo_final_plansys/pddl/domain.pddl) en PDDL con las siguientes características:

#### Tipos

Se definen 5 tipos diferentes:
- robot
- location
- person
- rubik
- book


### Predicados

Se definen 8 predicados:
- (robot_at ?r - robot ?l - location) : establece la posición del robot
- (rubik_at ?k - rubik ?l - location) : estable la posición del cubo de rubik
- (book_at ?b - book ?l - location) : establece la posición del libro
- (person_at ?p - person ?l - location) : estable la posición de la persona
- (rubik_solved ?k - rubik) : indica si un cubo de rubik 'k' está resuelto
- (book_found ?b - book) : indica si el libro 'b' se ha encontrado
- (person_attended ?p - person) : indica si la persona 'p' ha sido atendida
- (connected ?l1 ?l2 - location) : establece la conexión entre dos ubicaciones

### Acciones

Con estos tipos y predicados se implementan 4 acciones durativas:
1. *move* : cambia la posición del robot entre dos ubicaciones conectadas
2. *solve_rubik* : establece `rubik_solved` en caso de que el robot y el cubo estén en la misma ubicación
3. *search_book* : establece `book_found` en caso de que el robot y el libro estén en la misma ubicación
4. *attend_visitors* : establece `person_attended` en caso de que el robot y la persona estén en la misma ubicación

## Ejercicio 3 - Acciones

Implementar las distintas acciones en PlanSys2 como nodos de BehaviorTree. Se pueden utilizar los nodos implementados en [plansys2_bt_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_bt_example) como ejemplo.

**Importante:** La acción de navegar para que el robot se mueva de un punto a otro debe utilizar el sistema de navegación de nav2. Es decir, esta acción deberá realizar llamadas a las acciones de nav2 para que el robot se mueva de un punto a otro, de una forma similar a la implementada en los ejemplos [plansys2_bt_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_bt_example) y [plansys2_patrol_navigation_example](https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_patrol_navigation_example).

El resto de acciones a realizar por el robot pueden ser sintéticas, donde la acción habrá terminado después de que haya pasado un tiempo determinado.


### Composición de acciones con nodos de BT
Las acciones definidas en el dominio PDDL son equivalentes a un árbol de ejecución de BT. Ese árbol puede utilizar varias sub-acciones más pequeñas, que se implementarán como nodos del árbol. Por ejemplo, la acción de "resolver el cubo de Rubik" puede consistir en la secuencia de sub-acciones [detectar estado del cubo --> coger el cubo --> resolver el cubo --> soltar el cubo en la mesa]. Cada una de estas sub-acciones serán implementadas como un nodo de BehvaiorTree.

Indicar cómo han sido definidas las acciones en el paquete y qué sub-acciones (nodos BT) han sido implementadas:

**Respuesta**

Hemos creado varias composiciones de acciones para poder realizar las tareas de la biblioteca. Un ejemplo de ello es la tarea de "solve_rubik.xml" que se compone de las siguientes acciones:

```xml
<root BTCPP_format="4" main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <OpenGripper    name="open_gripper"/>
           <ApproachObject name="approach_object" />
           <CloseGripper   name="close_gripper"/>
           <SolveRubik     name="solve_rubik"/>
           <OpenGripper    name="open_gripper"/>
       </Sequence>
    </BehaviorTree>
</root>
```
En este caso, la tarea de "solve_rubik" se compone de las siguientes acciones:

- [OpenGripper](trabajo_final_plansys/src/behavior_tree_nodes/OpenGripper.cpp)
- [ApproachObject](trabajo_final_plansys/src/behavior_tree_nodes/ApproachObject.cpp)
- [CloseGripper](trabajo_final_plansys/src/behavior_tree_nodes/CloseGripper.cpp)
- [SolveRubik](trabajo_final_plansys/src/behavior_tree_nodes/SolveRubik.cpp)

Y cada una de estas acciones se implementa como un nodo de BT.

### Cada Nodo de BT

Dentro de un nodo de BT hace falta obligatoriamente tener una función `tick` que se encargará de ejecutar la acción y la función `halt` que se encargará de parar la acción. En el caso de que la acción no se pueda parar, se puede dejar vacía. La logica del codigo es mas o menos igual a la de un nodo normal de plasys2. En BT para acanar y dar informacion del Nodo se puede devolver un `BT::NodeStatus::SUCCESS` si la acción ha sido completada con exito, `BT::NodeStatus::FAILURE` si ha fallado y `BT::NodeStatus::RUNNING` si la acción sigue en ejecución.

Un ejemplo de un nodo de BT sería el siguiente:

```cpp

namespace trabajo_final_plansys
{

SolveRubik::SolveRubik(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), state_(CRUZ_BLANCA)
{
}

void
SolveRubik::halt()
{
  std::cout << "SolveRubik halt" << std::endl;
}

BT::NodeStatus
SolveRubik::tick()
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  if (!config().blackboard->get("node", node)) {
    RCLCPP_ERROR(node->get_logger(),
    "Failed to get 'node' from the blackboard");
  }

  srand(time(NULL));
  float detected_;
  detected_ = static_cast<float>(std::rand())/ static_cast<float>(RAND_MAX);

  if (detected_ < 0.9){
    switch (state_)
    {
    case CRUZ_BLANCA:
      RCLCPP_INFO(node->get_logger(), "MADE THE WHITE CROSS");
      state_ = ESQUINA_BLANCA;
      break;
    case ESQUINA_BLANCA:
      RCLCPP_INFO(node->get_logger(), "PLACED WHITE CORNERS");
    /* 
      CODIGO SIMPLIFICADO
     */

    case CUBO_COMPLETO:
      RCLCPP_INFO(node->get_logger(), "SOLVED THE RUBIK'S CUBE");
      return BT::NodeStatus::SUCCESS;
 
    default:
      RCLCPP_INFO(node->get_logger(), "Unknown state");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node->get_logger(), "I dont know how to solve Rubik's Cube :(");
  return BT::NodeStatus::FAILURE;
}

}  // namespace trabajo_final_plansys

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_final_plansys::SolveRubik>("SolveRubik");
}
```

Esta parte del código se encarga de registrar el nodo en el sistema de BT para que pueda ser utilizado.
```cpp
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_final_plansys::SolveRubik>("SolveRubik");
}
```

### Controlador

Una vez definidas las acciones, un [controlador](trabajo_final_plansys/src/library_controller_node.cpp) gestiona cuando se debe ejecutar cada acción mediante la definición de diferentes goals:

```cpp
// End when no goals left
if (goals_.empty()) {
  state_ = ENDING;
  break;
}

// set new goal
actual_goal_ = goals_.back();
problem_expert_->setGoal(plansys2::Goal("(and " + actual_goal_ + " )"));

// create new plan with goal
auto domain = domain_expert_->getDomain();
auto problem = problem_expert_->getProblem();
auto plan = planner_client_->getPlan(domain, problem);
```

Este snippet de código se ejecuta siempre que un nodo devuelve `SUCCESS` o `FAILURE` para poder replanificar:
- En caso de que devuelva **fallo**, la meta del plan sigue siendo la misma.
- En caso de que devuelva **éxito**, la meta del plan cambia para poder seguir iterando entre las metas buscadas.


## Ejercicio 4 - Planner
Utilizar un planificador distinto a POPF o TFD para generar los planes. Para conseguir esto es necesario implementar un plugin para poder llamar al planificador elegido desde PlanSys2. Este plugin se debe crear en un **paquete de ROS 2 aparte**, y consistirá en una clase que herede de [PlanSolverBase](https://github.com/PlanSys2/ros2_planning_system/blob/rolling/plansys2_core/include/plansys2_core/PlanSolverBase.hpp). Se puede utilizar la implementación del [plugin de POPF](https://github.com/PlanSys2/ros2_planning_system/tree/rolling/plansys2_popf_plan_solver) como referencia.

**Nota:** Si es posible, se deberá implementar un plugin para el planificador escogido por el grupo en el trabajo de PDDL. El planificador deberá soportar al menos `durative-actions`, por lo que si se escogió un planificador no compatible tendréis que elegir otro para implementar el plugin de PlanSys2.


Plugin escogido y detalles de cómo poder usarlo en plansys:

*[Respuesta]*


## Vídeo final
Para finalizar, se debe incluir un vídeo del sistema funcionando, en el que se pueda apreciar el desarrollo realizado.


### COMANDOS EJECUCIÓN FINAL

Para que el programa funcione correctamente, se deben ejecutar los siguientes comandos:

Terminal 1:

Lanzamos nuestro controlador y a la vez lanzamos gazebo con el mundo de la biblioteca:
```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch trabajo_final_plansys trabajo_final_plansys_launch.py 
```

Terminal 2:

Lanzamos el sistema de NAV2:
```bash
cd <ros2-workspace>
source install/setup.bash
ros2 launch kobuki navigation_sim.launch.py map:=src/kobuki/maps/aws_bookstore.yaml 
```

Terminal 3:

Lanzamos el los nodos:
```bash
cd <ros2-workspace>
source install/setup.bash
ros2 run trabajo_final_plansys library_controller_node 
```

Terminal 4 (opcional):

Para ver los plugins de PlanSys2:
```bash
rqt
```


**Vídeo**


https://github.com/user-attachments/assets/e53534b6-c863-4cac-b03d-f919722aff35




## Presentación
Se realizará una presentación de 10min del trabajo en clase y deberéis añadir también al repositorio los materiales (slides, vídeos, etc.) utilizados en la presentación.