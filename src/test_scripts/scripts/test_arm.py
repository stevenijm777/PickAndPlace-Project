#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

def load_gazebo_models():
    """Carga la mesa y tres cajas en Gazebo"""
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples') + "/models/"

    # Cargar modelos de la mesa y las cajas
    table_xml = open(model_path + "cafe_table/model.sdf", 'r').read()
    block_xml = open(model_path + "block/model.urdf", 'r').read()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    # Poses de la mesa y las cajas
    table_pose = Pose(position=Point(x=0.75, y=0.0, z=0.0))
    block1_pose = Pose(position=Point(x=0.6, y=0.1, z=0.7725))
    block2_pose = Pose(position=Point(x=0.6, y=0.3, z=0.7725))
    block3_pose = Pose(position=Point(x=0.6, y=-0.1, z=0.7725))

    # Spawn de la mesa
    try:
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn SDF service call failed: {e}")

    # Spawn de las cajas
    try:
        spawn_urdf("block1", block_xml, "/", block1_pose, "world")
        spawn_urdf("block2", block_xml, "/", block2_pose, "world")
        spawn_urdf("block3", block_xml, "/", block3_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn URDF service call failed: {e}")

def delete_gazebo_models():
    """Elimina los modelos de Gazebo"""
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model("cafe_table")
        delete_model("block1")
        delete_model("block2")
        delete_model("block3")
    except rospy.ServiceException as e:
        print(f"Delete Model service call failed: {e}")

def main():
    """Carga los modelos en Gazebo y espera que MoveIt los manipule"""
    rospy.init_node("load_gazebo_models")

    # Cargar modelos en Gazebo
    rospy.loginfo("Cargando modelos en Gazebo...")
    load_gazebo_models()

    # Elimina los modelos al cerrar
    rospy.on_shutdown(delete_gazebo_models)

    rospy.loginfo("Modelos cargados. Usa MoveIt para planificar rutas.")
    rospy.spin()

if __name__ == "__main__":
    main()