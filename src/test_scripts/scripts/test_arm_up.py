#!/usr/bin/env python
import rospy
from intera_interface import Limb

def move_arm_up():
    rospy.init_node('test_arm_up')
    limb = Limb('right')  # Cambia a 'left' si usas el otro brazo
    joint_angles = {
        'right_j0': 0.0,   # Rotación base
        'right_j1': -1.0,  # Levantar hacia arriba
        'right_j2': 0.0,   # Extender hacia adelante
        'right_j3': 2.0,   # Doblado del codo
        'right_j4': 0.0,   # Rotación muñeca
        'right_j5': 0.0,   # Inclinación muñeca
        'right_j6': 0.0    # Rotación final
    }
    rospy.loginfo("Moving arm to up position...")
    limb.move_to_joint_positions(joint_angles, timeout=5.0)
    rospy.loginfo("Arm should now be in the up position.")

if _name_ == '_main_':
    try:
        move_arm_up()
    except rospy.ROSInterruptException:
        pass