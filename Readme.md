
# Proyecto Pick and Place

Este proyecto es una simulación de un sistema *Pick and Place* utilizando ROS, Gazebo y un brazo robótico Sawyer, con el objetivo de mover objetos en un entorno simulado. A continuación se describen los pasos para configurar y ejecutar el proyecto.

## Instalación

1. **Instalar Ubuntu**
   
   Primero, asegúrate de tener Ubuntu instalado en tu sistema. Puedes seguir las instrucciones oficiales de instalación para ROS Noetic:

   [Instalación de ROS Noetic en Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)

2. **Clonar el repositorio del proyecto**

   Navega al directorio de inicio y clona el repositorio en tu máquina local:

   ```bash
   cd ~
   git clone https://github.com/stevenijm777/PickAndPlace-Project.git
   ```

3. **Compilar el proyecto**

   Luego, compila el proyecto utilizando `catkin_make`:

   ```bash
   cd PickAndPlace-Project
   catkin_make
   ```

4. **Configurar el entorno de trabajo**

   Fuente el archivo de configuración del espacio de trabajo para configurar el entorno:

   ```bash
   source ~/PickAndPlace-Project/devel/setup.bash
   ```

   Para que este paso se ejecute automáticamente cada vez que abras una terminal, agrega el comando al final de tu archivo `~/.bashrc`:

   ```bash
   echo "source ~/PickAndPlace-Project/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

5. **Abrir Visual Studio Code**

   Abre Visual Studio Code en el directorio del proyecto:

   ```bash
   code .
   ```

6. **Configurar la ruta de trabajo**

   Cambia la ruta en el archivo `src/test_scripts/scripts/test_arm.py`. Busca la línea que contiene:

   ```python
   miRuta = "/home/steven/catkin_ws/src/PickAndPlace-Project"
   ```

   y cámbiala a la ruta correcta según tu sistema.

7. **Clonar el repositorio del demo del transportador**

   Navega al directorio `src` y clona el repositorio del *conveyor_demo*:

   ```bash
   cd src
   git clone --recurse-submodules https://github.com/rokokoo/conveyor_demo.git
   ```

## Dependencias

1. **Instalar dependencias de OpenCV**

   - Instala el paquete `cv-bridge` para convertir imágenes de ROS a OpenCV:

     ```bash
     sudo apt-get install ros-noetic-cv-bridge
     ```

   - Instala las bibliotecas de Python necesarias:

     ```bash
     sudo apt install python3-pip
     pip install opencv-python numpy
     ```

     **Dependencias instaladas:**

     - `cv-bridge`: Para convertir imágenes de ROS a OpenCV.
     - `opencv-python`: Para procesar imágenes.
     - `numpy`: Para manejar matrices de imagen.

## Configuración y ejecución

1. **Compilar el espacio de trabajo**

   En la terminal, navega al directorio del espacio de trabajo y compílalo:

   ```bash
   cd ~/ros_ws
   catkin_make
   ```

2. **Cargar configuraciones del espacio de trabajo**

   Fuente el archivo de configuración del espacio de trabajo:

   ```bash
   source devel/setup.bash
   ```

3. **Ejecutar el script del brazo robótico**

   Desde el directorio `PickAndPlace-Project`, ejecuta el script necesario:

   ```bash
   ./src/intera_sdk/intera.sh sim
   ```

4. **Lanzar Gazebo con la banda transportadora y Sawyer**

   Ejecuta el siguiente comando para lanzar Gazebo con el entorno simulado:

   ```bash
   roslaunch test_scripts simple_pick_and_place.launch
   ```

5. **Permitir comunicación entre RViz y MoveIt**

   En una nueva terminal, ejecuta:

   ```bash
   rosrun intera_interface enable_robot.py -e
   rosrun intera_interface joint_trajectory_action_server.py
   ```

6. **Lanzar RViz con MoveIt**

   Ejecuta el siguiente comando para lanzar RViz con la configuración de MoveIt:

   ```bash
   roslaunch sawyer_moveit_config demo.launch
   ```

7. **Cámara**

   Ejecuta el siguiente comando en otra terminal para visualizar la cámara:

   ```bash
   rosrun rqt_image_view rqt_image_view
   ```

   En el campo de imagen selecciona `camera/image_raw`.

8. **Ejecutar la trayectoria**

   Ejecuta el siguiente comando para mover el brazo a la posición de recogida:
   Ejecutar al mismo tiempo que el paso 10 para una buena sincronizacion

   ```bash
   rosrun planning run 4 - go to pick
   ```

9. **Ejecutar la trayectoria para diferentes cajas**

   Dependiendo del color del objeto, ejecuta:

   ```bash
   rosrun planning run 1 - go to Caja 1
   rosrun planning run 2 - go to Caja 2
   rosrun planning run 3 - go to Caja 3
   ```

10. **Ejecutar la banda transportadora**

    Finalmente, ejecuta el siguiente comando para controlar la banda transportadora:

    ```bash
    rosrun planning banda
    ```


# Guía para solucionar errores comunes en el proyecto PickAndPlace

A continuación, se detallan algunos de los errores comunes que podrías encontrar durante la configuración y ejecución del proyecto PickAndPlace, junto con sus soluciones correspondientes.

## Error 1: `'/usr/bin/env: ‘python’: No such file or directory'`

Este error ocurre cuando el sistema no encuentra el ejecutable de Python. Para solucionarlo, crea un enlace simbólico de `python3` a `python` de la siguiente manera:

1. Verifica la versión de Python 3 instalada:
   ```bash
   python3 --version
   ```
2. Crea el enlace simbólico:
   ```bash
   sudo ln -s /usr/bin/python3 /usr/bin/python
   ```

## Error 2: `CV_LOAD_IMAGE_UNCHANGED not declared in this scope`

Este error se presenta cuando se intenta utilizar una constante obsoleta de OpenCV en el archivo `head_interface.cpp` de `sawyer_simulator`. Para solucionarlo, reemplaza la constante `CV_LOAD_IMAGE_UNCHANGED` por `cv::IMREAD_UNCHANGED` en la línea correspondiente del código.

1. Abre el archivo `head_interface.cpp`:
   ```bash
   nano ~/catkin_ws/src/PickAndPlace-Project/src/sawyer_simulator/sawyer_gazebo/src/head_interface.cpp
   ```
2. Reemplaza `CV_LOAD_IMAGE_UNCHANGED` por `cv::IMREAD_UNCHANGED`:
   ```cpp
   cv_ptr->image = cv::imread(img_path, cv::IMREAD_UNCHANGED);
   ```

## Error 3: Problema con la instalación de dependencias

Si encuentras problemas al instalar dependencias, como `opencv-python`, `numpy` o `cv-bridge`, sigue estos pasos:

1. Instala `cv-bridge`:
   ```bash
   sudo apt-get install ros-noetic-cv-bridge
   ```

2. Instala las dependencias de Python:
   ```bash
   sudo apt install python3-pip
   pip install opencv-python numpy
   ```

## Error 4: `Package 'demo_world' not found`

Este error ocurre cuando se intenta ejecutar el paquete `demo_world` que no está presente en el espacio de trabajo. Para solucionarlo, sigue los siguientes pasos:

1. Clona el repositorio de `conveyor_demo`:
   ```bash
   cd ~/catkin_ws/src
   git clone --recurse-submodules https://github.com/rokokoo/conveyor_demo.git
   ```
2. Luego, intenta de nuevo ejecutar el lanzamiento de Gazebo.

## Error 5: `Service call failed: service [/gazebo/spawn_urdf_model] responded with an error`

Este error puede ocurrir si el modelo de robot no se carga correctamente. Asegúrate de que todos los paquetes necesarios estén correctamente instalados y que el modelo URDF esté configurado correctamente.

1. Verifica que los paquetes del robot estén correctamente clonados:
   ```bash
   git clone https://github.com/RethinkRobotics/sawyer_robot.git
   git clone https://github.com/RethinkRobotics/intera_common.git
   ```
2. Si aún persiste el error, revisa el archivo de log para obtener más detalles sobre el error:
   ```bash
   cat /home/steven/.ros/log/c42bf382-df69-11ef-842a-b1dd58237c57/urdf_spawner-5.log
   ```

## Error 6: Problemas con `dpkg` y la instalación de dependencias

Si ves el error `dpkg was interrupted`, puedes solucionarlo con el siguiente comando:

1. Ejecuta:
   ```bash
   sudo dpkg --configure -a
   ```
2. Luego, actualiza las dependencias:
   ```bash
   sudo apt-get update
   sudo apt install ros-noetic-industrial-core ros-noetic-object-recognition-msgs
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```

## Recomendaciones adicionales

- Si algún paquete o repositorio presenta problemas de compatibilidad o faltan archivos, puedes usar `git status` para ver qué archivos han sido modificados o eliminados.
- Si después de eliminar algún paquete o archivo, el proyecto no compila correctamente, puedes intentar reinstalar los submódulos del repositorio:
   ```bash
   git submodule update --init --recursive
   ```

## Otros pasos importantes

1. Para ejecutar el entorno de simulación de `PickAndPlace-Project`:
   ```bash
   bash src/PickAndPlace-Project/src/intera_sdk/intera.sh sim
   ```

2. Para permitir la comunicación entre RViz y MoveIt:
   ```bash
   rosrun intera_interface enable_robot.py -e
   rosrun intera_interface joint_trajectory_action_server.py
   ```

3. Para ejecutar Gazebo con la banda y el robot Sawyer:
   ```bash
   roslaunch test_scripts simple_pick_and_place.launch
   ```

