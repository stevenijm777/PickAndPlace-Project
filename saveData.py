import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState

# Ruta al archivo .bag
bag_file = "2025-01-19-22-48-18.bag"

# Almacenar datos
time = []
positions = []
velocities = []
efforts = []

# Leer datos del bag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/robot/joint_states']):
        time.append(t.to_sec())
        positions.append(msg.position)  # Posiciones angulares
        velocities.append(msg.velocity)  # Velocidades angulares
        efforts.append(msg.effort)  # Torques aplicados

# Normalizar tiempo
time = [t - time[0] for t in time]

# Graficar datos
for i, joint_name in enumerate(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']):
    plt.figure()
    plt.plot(time, [p[i] for p in positions], label="Position (rad)")
    plt.plot(time, [v[i] for v in velocities], label="Velocity (rad/s)")
    plt.title(f"{joint_name} Data")
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.legend()
    plt.grid()
    plt.show()