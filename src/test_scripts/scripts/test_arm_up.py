#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel
from intera_interface import Limb

class SimplePickAndPlace:
    def __init__(self):
        rospy.init_node("simple_pick_and_place")
        self.limb = Limb("right")  # Cambia "right" por "left" si usas el otro brazo
        self.hover_distance = 0.15  # Altura de "hover" sobre el objeto

    def load_models(self):
        """Carga la mesa y la caja roja en Gazebo"""
        model_path = rospkg.RosPack().get_path('sawyer_sim_examples') + "/models/"
        table_xml = open(model_path + "cafe_table/model.sdf", 'r').read()
        block_xml = open(model_path + "block/model.urdf", 'r').read()

        table_pose = Pose(position=Point(x=0.75, y=0.0, z=0.0))
        block_pose = Pose(position=Point(x=0.7, y=0.1, z=0.7725))
        
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")

        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_urdf("block", block_xml, "/", block_pose, "world")

    def move_to_pose(self, pose):
        """Mueve el brazo a una pose específica"""
        joint_angles = self.limb.ik_request(pose, "right_gripper_tip")
        if joint_angles:
            self.limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No se encontró solución de cinemática inversa para la pose.")

    def run(self):
        """Ejecución básica: mover a hover y luego a una posición"""
        hover_pose = Pose(
            position=Point(x=0.7, y=0.1, z=1.0),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        )
        target_pose = Pose(
            position=Point(x=0.7, y=0.1, z=0.7725),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        )
        
        self.move_to_pose(hover_pose)
        self.move_to_pose(target_pose)

if __name__=='__main__':
    demo = SimplePickAndPlace()
    demo.load_models()
    demo.run()