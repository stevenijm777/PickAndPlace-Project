#!/usr/bin/env python3
import tf
import rospy
import rospkg
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_conveyor.srv import ConveyorBeltControl

class CubeSpawner():
    def __init__(self) -> None:
        self.rospack = rospkg.RosPack()
        # Ruta a los bloques
        self.path = "/home/coflores/ros/src/PickAndPlace-Project/src/sawyer_simulator/sawyer_sim_examples/models/block/"
        # Ruta al modelo del bin
        self.path_bin = "/home/coflores/ros/src/PickAndPlace-Project/src/sawyer_simulator/sawyer_sim_examples/models/bin.urdf"

        self.cubes = [
            self.path + "red_cube.urdf",
            self.path + "green_cube.urdf",
            self.path + "blue_cube.urdf"
        ]
        self.col = 0

        # Inicializar servicios de Gazebo
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/conveyor/control")

        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.conveyor_control = rospy.ServiceProxy("/conveyor/control", ConveyorBeltControl)

        # Configurar la cámara
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, self.image_callback)
        
        self.block_detected = False
        self.block_position = None

        rospy.loginfo("CubeSpawner initialized.")

    def spawn_bin(self):
        """Carga los bins en Gazebo en forma de L."""
        with open(self.path_bin, "r") as bin_file:
            bin_urdf = bin_file.read()
        
        bin_positions = [
            Pose(Point(x=0.7, y=-0.25, z=0.09), Quaternion(0, 0, 0, 1)),
            Pose(Point(x=0.7, y=0.35, z=0.09), Quaternion(0, 0, 0, 1)),
            Pose(Point(x=1.3, y=0.35, z=0.09), Quaternion(0, 0, 0, 1))
        ]
        
        for i, pose in enumerate(bin_positions):
            self.sm(f"bin_{i+1}", bin_urdf, "", pose, "world")
            rospy.loginfo(f"Bin {i+1} spawned at {pose.position.x}, {pose.position.y}, {pose.position.z}")

    def spawnModel(self):
        """Carga un cubo en Gazebo."""
        cube = self.cubes[self.col]
        with open(cube, "r") as f:
            cube_urdf = f.read()

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=2, y=-1.0, z=2), orient)  # Ajustada la posición
        
        rospy.loginfo(f"Spawning cube {self.col + 1} at {pose.position.x}, {pose.position.y}, {pose.position.z}")
        self.sm(f"cube_{self.col + 1}", cube_urdf, "", pose, "world")

        self.col = (self.col + 1) % len(self.cubes)
        rospy.sleep(1)

    def deleteModel(self):
        """Elimina todos los cubos generados."""
        for i in range(len(self.cubes)):
            try:
                self.dm(f"cube_{i+1}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to delete cube_{i+1}: {e}")
        rospy.sleep(1)

    def image_callback(self, msg):
        """Procesa la imagen de la cámara y detecta el bloque en la banda."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 20 and h > 20:
                    cx = x + w // 2
                    cy = y + h // 2
                    rospy.loginfo(f"Bloque detectado en: X={cx}, Y={cy}")
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    if 250 < cx < 350:  # Ajusta la región de detección
                        self.block_detected = True
                        self.block_position = (cx, cy)
                        self.stop_conveyor()

            cv2.imshow("Block Detection", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("Error en el procesamiento de la imagen: %s", e)

    def stop_conveyor(self):
        """Detiene la banda transportadora."""
        try:
            rospy.loginfo("Deteniendo la banda transportadora...")
            self.conveyor_control(0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Error al detener la banda: {e}")

    def start_conveyor(self):
        """Reinicia la banda transportadora."""
        try:
            rospy.loginfo("Arrancando la banda transportadora...")
            self.conveyor_control(40)
        except rospy.ServiceException as e:
            rospy.logerr(f"Error al iniciar la banda: {e}")

    def shutdown_hook(self):
        """Limpia modelos al apagar."""
        self.deleteModel()
        rospy.loginfo("Shutting down and cleaning up.")

if __name__ == "__main__":
    rospy.init_node("spawn_objects")
    cs = CubeSpawner()
    cs.spawn_bin()

    rospy.on_shutdown(cs.shutdown_hook)
    
    while not rospy.is_shutdown():
        if not cs.block_detected:
            cs.start_conveyor()
            rospy.sleep(2)

        if cs.block_detected:
            rospy.loginfo(f"Bloque detectado en posición {cs.block_position}")
            cs.deleteModel()
            cs.spawnModel()
            cs.block_detected = False
        rospy.sleep(1)
