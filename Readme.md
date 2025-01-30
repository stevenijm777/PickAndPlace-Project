# README - Configuración y Ejecución de Sawyer en ROS

Este documento describe los pasos necesarios para configurar y ejecutar la simulación del robot Sawyer en ROS, incluyendo la comunicación con RViz y MoveIt, así como la ejecución de trayectorias y control de los actuadores.

## 1. Configuración del Espacio de Trabajo

### Navegar al directorio de trabajo y compilar el workspace:
```bash
cd ~/ros_ws
catkin_make
```

### Cargar las configuraciones del espacio de trabajo:
```bash
source devel/setup.bash
```

## 2. Ejecutar el script de configuración de Sawyer
Ejecutar el script dentro del directorio del workspace:
```bash
./src/intera_sdk/intera.sh sim
```

## 3. Lanzar la simulación en Gazebo
```bash
roslaunch sawyer_gazebo sawyer_world.launch
roslaunch test_scripts simple_pick_and_place.launch
```

## 4. Permitir comunicación entre RViz y MoveIt
Habilitar el robot y el servidor de trayectorias:
```bash
rosrun intera_interface enable_robot.py -e
rosrun intera_interface joint_trajectory_action_server.py
```

## 5. Lanzar RViz y MoveIt
```bash
roslaunch sawyer_sim_examples sawyer_pick_and_place_demo.launch
roslaunch sawyer_moveit_config my_connect.launch
roslaunch sawyer_moveit_config demo.launch
```

## 6. Ejecutar trayectorias
Ejecutar una trayectoria planeada:
```bash
rosrun planning run 4
```
Controlar el gripper:
```bash
rosrun intera_interface control_gripper.py
```
Enviar comandos al conveyor:
```bash
rosservice call /conveyor/control "{power: 20}"
```

## 7. Control del gripper
Publicar en el tópico de control del gripper:
```bash
rostopic pub -1 /gripper_control std_msgs/Int32 "data: 1"
```
Ejecutar script de control del gripper:
```bash
rosrun intera_interface control_gripper.py
```

## 8. Registro de datos con rosbag
```bash
rosbag record /robot/joint_states
```

## 9. Configuración de permisos de ejecución
```bash
chmod +x ~/ros_ws/src/test_scripts/scripts/test_arm_.py
chmod +x ~/ros_ws/src/planning/src/test_gripper.cpp
```

## 10. Control del Gripper Eléctrico
Abrir el gripper izquierdo:
```bash
rostopic pub -1 /robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command std_msgs/Float64 "data: 1.0"
```
Cerrar el gripper derecho:
```bash
rostopic pub -1 /robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command std_msgs/Float64 "data: -1.0"
```

## 11. Subir cambios al repositorio Git
```bash
git push -u origin main
```

