import numpy as np

def dh_transform(theta, d, a, alpha):
    """Crea la matriz de transformación homogénea usando los parámetros DH."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_params):
    """Calcula la cinemática directa dado un conjunto de parámetros DH."""
    T = np.eye(4)  # Matriz identidad 4x4
    for params in dh_params:
        T = np.dot(T, dh_transform(*params))  # Producto acumulativo de transformaciones
    return T

def DH(q1,q2,q3,q4,q5,q6,q7=1.0):
    # Parámetros D-H del robot (theta, d, a, alpha)
    dh_params = [
        (q1, 0.317, 0.081, np.deg2rad(-90)),  # Junta 1
        (q2, 0.1925, 0, np.deg2rad(-90)),       # Junta 2
        (q3, 0.4, 0, np.deg2rad(-90)), # Junta 3
        (q4, 0.1685, 0, np.deg2rad(-90)),# Junta 4
        (q5, 0.4, 0, np.deg2rad(-90)),   # Junta 5
        (q6, 0.1363, 0, np.deg2rad(-90)),        # Junta 6
        (q7, 0.1338, 0, 0)]
    return dh_params

q1 = 0.577
q2 = -0.548
q3 = 1.596
q4 = -2.317
q5 = -1.638
q6 = 1.191
q7 = 1.0

dh_params = []
# Cinemática directa
T = forward_kinematics(dh_params)

# Posición y orientación
position = T[:3, 3]
orientation = T[:3, :3]

print("Posición (x, y, z):", position)
print("Orientación (matriz de rotación):\n", orientation)
