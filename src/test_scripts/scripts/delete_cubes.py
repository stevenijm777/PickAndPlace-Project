#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel

def delete_cube(cube_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model(cube_name)
        if resp.success:
            rospy.loginfo(f"Cubo {cube_name} eliminado correctamente.")
        else:
            rospy.logwarn(f"No se pudo eliminar {cube_name}.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al eliminar {cube_name}: {e}")

if __name__ == "__main__":
    rospy.init_node("delete_cubes_node")

    # Lista de cubos a eliminar manualmente
    cube_names = ["cube_red_1", "cube_green_1", "cube_blue_1"]

    for cube in cube_names:
        delete_cube(cube)

    rospy.loginfo("Proceso de eliminación completado.")
