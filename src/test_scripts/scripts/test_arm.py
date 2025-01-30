#!/usr/bin/env python3
import tf
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8MultiArray

class CubeSpawner():
    def __init__(self):
        self.rospack = rospkg.RosPack()
        # Mi directiorio
        miRuta = "/home/steven/catkin_ws/src/PickAndPlace-Project"
        # Ruta a los bloques
        self.path = miRuta+ "/src/sawyer_simulator/sawyer_sim_examples/models/block/"
        # Ruta al modelo del bin
        self.path_bin = miRuta+"/src/sawyer_simulator/sawyer_sim_examples/models/bin.urdf"

        self.cubes = []
        self.cubes.append(self.path + "red_cube.urdf")
        self.cubes.append(self.path + "green_cube.urdf")
        self.cubes.append(self.path + "blue_cube.urdf")
        self.col = 0
        self.spawned_cubes = [] # lista de nombres de cubos generados


        # Inicializar servicios de Gazebo
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/get_model_state")

        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        # Publicador del estado de cubos
        self.cube_pub = rospy.Publisher("current_cube_blocks", Int8MultiArray, queue_size=10)

        rospy.loginfo("CubeSpawner initialized.")

    def spawn_bin(self):
        """Carga los bins en Gazebo en forma de L."""

        with open(self.path_bin, "r") as bin_file:
            bin_urdf = bin_file.read()

        bin_positions = [

            Point(x=-0.8, y=-0.25, z=0.05),  # Bin 1 (posición base)
            Point(x=-0.8, y=0.1, z=0.05),  # Bin 2 (al lado de Bin 1 en Y)
            Point(x=0.8, y=0.04, z=0.05)  # Bin 3 (perpendicular en X)
            ]
        
        for i, pos in enumerate(bin_positions):
            pose = Pose(pos, Quaternion(0, 0, 0, 1))
            self.sm(f"bin_{i+1}", bin_urdf, "", pose, "world")
            rospy.loginfo(f"Bin {i+1} spawned at position {pos.x}, {pos.y}, {pos.z}")

    def checkModel(self, name):

        # Verifica si un modelo especifico existe un Gazebo
        res = self.ms(name, "world")
        return res.success

    def getPosition(self, name):
        res = self.ms(name, "world")
        return res.pose.position.z


    def spawnModel(self):
        """Genera un cubo en Gazebo con nombre único y en secuencia de colores."""

        current_color = self.cube_colors[(self.cube_counter - 1) % len(self.cube_colors)]
        cube_path = f"{self.path}{current_color}_cube.urdf"

        with open(cube_path, "r") as f:
            cube_urdf = f.read()


        # Crear nombre unico para cada cubo 
        cube_name = f"cube_{len(self.spawned_cubes)+ 1}"

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=1.8, y=-0.768, z=0.75), orient)  # Ubicación del cubo

        # Verificar si ya hay un cubo en la posicion inicial
        for name in self.spawned_cubes:
            if self.getPosition(name) >= 0.65:
                #rospy.loginfo("Ya hay un cubo en la altura inicial. No se genera otra")
                return
        # Si no hay un cubo en la altura inicial, se genera uno nuevo
        self.sm(cube_name, cube_urdf, "", pose, "world")
        self.spawned_cubes.append(cube_name)
            rospy.loginfo(f"Spawning {cube_name}...")
            self.sm(cube_name, cube_urdf, "", pose, "world")
            rospy.loginfo(f"{cube_name} spawned successfully.")

            self.publish_cube_list(self.cube_counter % len(self.cube_colors))
            self.cube_counter += 1
        else:
            rospy.logwarn(f"{cube_name} already exists, skipping...")

        rospy.sleep(15)

    def shutdown_hook(self):
        rospy.loginfo("Shutting down and cleaning up.")
        for i in range(1, self.cube_counter):
            for color in self.cube_colors:
                cube_name = f"cube_{color}_{i}"
                try:
                    self.dm(cube_name)
                    rospy.loginfo(f"Deleted {cube_name}")
                except rospy.ServiceException:
                    rospy.logwarn(f"Could not delete {cube_name}, it may not exist.")

if __name__ == "__main__":
    rospy.init_node("spawn_objects")
    cs = CubeSpawner()
    cs.spawn_bin()

    rospy.on_shutdown(cs.shutdown_hook)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        # estado incial
        cs.spawnModel()
        r.sleep()
