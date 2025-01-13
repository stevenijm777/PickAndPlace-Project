#!/usr/bin/env python

import rospy
from intera_interface import Limb

def simple_move():
    rospy.init_node('test_arm_movement')
    limb = Limb('right')  # Asegúrate de que el brazo sea 'right' o el correcto para tu configuración
    joint_angles = {'right_j0': 0.0, 'right_j1': -1.0, 'right_j2': 0.0, 'right_j3': 2.0, 'right_j4': 0.0, 'right_j5': -1.5, 'right_j6': 0.0}
    limb.move_to_joint_positions(joint_angles)

if __name__ == "__main__":
    try:
        simple_move()
    except rospy.ROSInterruptException:
        pass