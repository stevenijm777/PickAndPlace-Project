#!/usr/bin/env python

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_objects_to_moveit():
    rospy.init_node("add_objects_to_moveit")
    scene = PlanningSceneInterface()

    rospy.sleep(2)  # Espera a que el nodo se inicialice

    # Añadir la mesa
    table_pose = PoseStamped()
    table_pose.header.frame_id = "base"
    table_pose.pose.position.x = 0.75
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = 0.35  # Altura de la mesa
    scene.add_box("table", table_pose, size=(0.5, 1.0, 0.7))  # Tamaño de la mesa

    # Añadir las cajas
    for i, y in enumerate([0.1, 0.3, -0.1]):  # Posiciones de las cajas
        block_pose = PoseStamped()
        block_pose.header.frame_id = "base"
        block_pose.pose.position.x = 0.6
        block_pose.pose.position.y = y
        block_pose.pose.position.z = 0.85  # Altura sobre la mesa
        scene.add_box(f"block{i+1}", block_pose, size=(0.05, 0.05, 0.05))  # Tamaño de las cajas

    rospy.loginfo("Objetos añadidos a MoveIt")

if __name__ == "__main__":
    add_objects_to_moveit()