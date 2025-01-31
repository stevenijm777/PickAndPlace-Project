#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

# Parámetros de la cámara
CAMERA_HEIGHT = 1.5  # Altura de la cámara en metros sobre la banda
CAMERA_FOV = 1.047  # Campo de visión horizontal en radianes (~60°)
IMAGE_WIDTH = 640  # Ancho de la imagen en píxeles
IMAGE_HEIGHT = 480  # Alto de la imagen en píxeles
BAND_WIDTH = 0.85  # Ancho de la banda transportadora en metros

def pixel_to_world(x_img, y_img):
    """
    Convierte coordenadas de la imagen (x_img, y_img) en coordenadas reales de Gazebo (X_gazebo, Y_gazebo).
    """
    # Calcular el factor de escala en metros por píxel
    scale_x = BAND_WIDTH / IMAGE_WIDTH  # Relación ancho banda - imagen
    scale_y = (2 * CAMERA_HEIGHT * np.tan(CAMERA_FOV / 2)) / IMAGE_HEIGHT  # Relación altura - imagen

    # Convertir coordenadas de píxeles a metros
    X_gazebo = (x_img - IMAGE_WIDTH / 2) * scale_x  # Centrar en el eje
    Y_gazebo = (y_img - IMAGE_HEIGHT / 2) * -scale_y  # Invertir eje Y para alinearlo con Gazebo
    Z_gazebo = 0  # La banda transportadora está en Z = 0

    return X_gazebo, Y_gazebo, Z_gazebo

def image_callback(msg):
    # Convertir imagen de ROS a OpenCV
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Convertir imagen a HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Definir rangos de color para detección
    lower_yellow = np.array([20, 100, 100])  # Amarillo en HSV
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Detectar contornos de bloques amarillos
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_yellow:
        if cv2.contourArea(contour) > 500:  # Filtrar ruido
            x, y, w, h = cv2.boundingRect(contour)
            X_gazebo, Y_gazebo, Z_gazebo = pixel_to_world(x + w//2, y + h//2)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 255), 2)  # Amarillo (BGR: 0,255,255)
            rospy.loginfo(f"Bloque AMARILLO en la imagen (x: {x}, y: {y}) → Gazebo (X: {X_gazebo:.2f}, Y: {Y_gazebo:.2f}, Z: {Z_gazebo})")

    # Detectar contornos de bloques verdes
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_green:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            X_gazebo, Y_gazebo, Z_gazebo = pixel_to_world(x + w//2, y + h//2)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            rospy.loginfo(f"Bloque VERDE en la imagen (x: {x}, y: {y}) → Gazebo (X: {X_gazebo:.2f}, Y: {Y_gazebo:.2f}, Z: {Z_gazebo})")

    # Detectar contornos de bloques azules
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_blue:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            X_gazebo, Y_gazebo, Z_gazebo = pixel_to_world(x + w//2, y + h//2)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            rospy.loginfo(f"Bloque AZUL en la imagen (x: {x}, y: {y}) → Gazebo (X: {X_gazebo:.2f}, Y: {Y_gazebo:.2f}, Z: {Z_gazebo})")

    # Mostrar la imagen procesada
    cv2.imshow("Camera View", cv_image)
    cv2.imshow("Yellow Mask", mask_yellow)
    cv2.imshow("Green Mask", mask_green)
    cv2.imshow("Blue Mask", mask_blue)
    cv2.waitKey(1)

# Inicializar nodo ROS
rospy.init_node('color_detector', anonymous=True)
rospy.Subscriber("/camera/image_raw", Image, image_callback)

rospy.spin()
cv2.destroyAllWindows()
