#!/usr/bin/env python3
import tf
import os
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion

class CubeSpawner():
    def __init__(self) -> None:
        self.rospack = rospkg.RosPack()
        # Ruta a los bloques
        self.path = "/home/stemjara/ros_ws/src/sawyer_simulator/sawyer_sim_examples/models/block/"
        # Ruta al modelo del bin
        self.path_bin = "/home/stemjara/ros_ws/src/sawyer_simulator/sawyer_sim_examples/models/bin.urdf"

        self.cubes = []
        self.cubes.append(self.path + "red_cube.urdf")
        self.cubes.append(self.path + "green_cube.urdf")
        self.cubes.append(self.path + "blue_cube.urdf")
        self.col = 0

        # Inicializar servicios de Gazebo
        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.loginfo("CubeSpawner initialized.")

    def spawn_bin(self):
        """Carga los bins en Gazebo en forma de L."""

        with open(self.path_bin, "r") as bin_file:
            bin_urdf = bin_file.read()

        # Tamaño de los bins
        bin_width = 0.45  # Y dimension más amplia
        bin_height = 0.4  # X dimension

        # Posiciones para formar una "L"
        bin_positions = [
            Point(x=-0.9, y=-0.25, z=0.09),  # Bin 1 (posición base)
            Point(x=-0.9, y=bin_width - 0.15, z=0.09),  # Bin 2 (al lado de Bin 1 en Y)
            Point(x=-0.9 - bin_height - 0.2, y=bin_width - 0.15, z=0.09)  # Bin 3 (perpendicular en X)
        ]

        for i, pos in enumerate(bin_positions):
            pose = Pose(pos, Quaternion(0, 0, 0, 1))  # Rotación en cero
            self.sm(f"bin_{i+1}", bin_urdf, "", pose, "world")
            rospy.loginfo(f"Bin {i+1} spawned at position {pos.x}, {pos.y}, {pos.z}")

    def checkModel(self):
        res = self.ms("cube", "world")
        return res.success

    def getPosition(self):
        res = self.ms("cube", "world")
        return res.pose.position.z

    def spawnModel(self):
        """Carga un cubo en Gazebo."""
        cube = self.cubes[self.col]
        with open(cube, "r") as f:
            cube_urdf = f.read()

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=0.8, y=-0.8, z=0.75), orient)  # Ubicación del cubo
        self.sm("cube", cube_urdf, "", pose, "world")

        if self.col < 2:  # Cambiar al siguiente cubo
            self.col += 1
        else:
            self.col = 0
        rospy.sleep(1)

    def deleteModel(self):
        """Elimina el cubo actual de Gazebo."""
        self.dm("cube")
        rospy.sleep(1)

    def shutdown_hook(self):
        """Limpia modelos al apagar."""
        self.deleteModel()
        rospy.loginfo("Shutting down and cleaning up.")

if __name__ == "__main__":
    print("Waiting for Gazebo services...")
    rospy.init_node("spawn_objects")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")
    
    r = rospy.Rate(15)
    cs = CubeSpawner()
    cs.spawn_bin()  # Cargar los bins en forma de L

    rospy.on_shutdown(cs.shutdown_hook)
    while not rospy.is_shutdown():
        if cs.checkModel() == False:
            cs.spawnModel()
        elif cs.getPosition() < 0.05:
            cs.deleteModel()
        r.sleep()
