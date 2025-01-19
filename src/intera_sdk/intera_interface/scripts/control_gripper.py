#!/usr/bin/env python

import rospy
from intera_interface import gripper as robot_gripper
from std_msgs.msg import Int32

def gripper_callback(msg):
    # Crear una instancia del gripper derecho
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Verificar si el gripper está conectado
    if not right_gripper.is_ready():
        rospy.loginfo("Gripper no está listo. Intentando calibrarlo...")
        right_gripper.calibrate()
        rospy.sleep(1.0)

    # Ejecutar la acción solicitada
    if msg.data == 1:
        rospy.loginfo("Abriendo la pinza...")
        right_gripper.open()
    elif msg.data == 0:
        rospy.loginfo("Cerrando la pinza...")
        right_gripper.close()
    else:
        rospy.logwarn("Comando desconocido recibido. Use 1 para abrir o 0 para cerrar.")

    rospy.loginfo("Acción completada.")

def main():
    rospy.init_node('gripper_control_node')
    rospy.Subscriber('/gripper_control', Int32, gripper_callback)
    rospy.loginfo("Nodo de control del gripper en espera de comandos...")
    rospy.spin()

if __name__ == '__main__':
    main()

