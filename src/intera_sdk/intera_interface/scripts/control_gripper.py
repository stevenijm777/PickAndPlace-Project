#!/usr/bin/env python

import rospy
from intera_interface import gripper as robot_gripper
import sys

def main():
    rospy.init_node('gripper_control_node')

    # Verificar que se haya pasado un argumento
    if len(sys.argv) != 2:
        rospy.logerr("Uso: rosrun tu_paquete gripper_control_node.py [1|0]")
        return

    # Obtener el argumento de la línea de comandos
    try:
        action = int(sys.argv[1])
    except ValueError:
        rospy.logerr("El argumento debe ser un número entero (1 para abrir, 0 para cerrar).")
        return

    if action not in [0, 1]:
        rospy.logerr("El argumento debe ser 1 (abrir) o 0 (cerrar).")
        return

    # Crear una instancia del gripper derecho
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Verificar si el gripper está conectado
    if not right_gripper.is_ready():
        rospy.loginfo("Gripper no está listo. Intentando calibrarlo...")
        right_gripper.calibrate()
        rospy.sleep(1.0)

    # Ejecutar la acción solicitada
    if action == 1:
        rospy.loginfo("Abriendo la pinza...")
        right_gripper.open()
    elif action == 0:
        rospy.loginfo("Cerrando la pinza...")
        right_gripper.close()

    rospy.sleep(1.0)
    rospy.loginfo("Control de la pinza completado.")

if __name__ == '__main__':
    main()
