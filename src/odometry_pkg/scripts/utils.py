# utils.py
import numpy as np
import rospy
import tf.transformations as tft
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class IMUListener:
    def _init_(self):
        # Initialize storage for each IMU value
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accel_filtered = {'x': 0.0, 'y': 0.0, 'z': 0.0} #Cambiar a matriz
        self.gyro_filtered = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Subscribe to each ROS topic published by the ESP32
        rospy.Subscriber("imu/yaw", Float32, self.yaw_callback)
        rospy.Subscriber("imu/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("imu/roll", Float32, self.roll_callback)

        rospy.Subscriber("imu/accel_x", Float32, self.ax_callback)
        rospy.Subscriber("imu/accel_y", Float32, self.ay_callback)
        rospy.Subscriber("imu/accel_z", Float32, self.az_callback)

        rospy.Subscriber("imu/gyro_x", Float32, self.gx_callback)
        rospy.Subscriber("imu/gyro_y", Float32, self.gy_callback)
        rospy.Subscriber("imu/gyro_z", Float32, self.gz_callback)

    # Callback functions for each topic
    def yaw_callback(self, msg): self.yaw = msg.data
    def pitch_callback(self, msg): self.pitch = msg.data
    def roll_callback(self, msg): self.roll = msg.data

    def ax_callback(self, msg): self.accel_filtered['x'] = msg.data
    def ay_callback(self, msg): self.accel_filtered['y'] = msg.data
    def az_callback(self, msg): self.accel_filtered['z'] = msg.data

    def gx_callback(self, msg): self.gyro_filtered['x'] = msg.data
    def gy_callback(self, msg): self.gyro_filtered['y'] = msg.data
    def gz_callback(self, msg): self.gyro_filtered['z'] = msg.data

def compute_quaternion(theta):
    return tft.quaternion_from_euler(0, 0, theta)

class VESCRPMListener: 
    def _init_(self):

        # Variable to store the most recent RPM
        self.rpm_value = 0

        # Subscribe to the RPM topic
        rospy.Subscriber("vesc/rpm", Int32, self.rpm_callback)

    def rpm_callback(self, msg):
        self.rpm_value = msg.data  # Update the value
        rospy.loginfo("Updated RPM: %d", self.rpm_value)

    def run(self):
        rate = rospy.Rate(10)  # Loop at 10 Hz
        while not rospy.is_shutdown():
            # Use the most recent rpm_value here
            rospy.loginfo("Current RPM: %d", self.rpm_value)
            rate.sleep()

def initialize_serial(port, baud_rate, timeout):
    """Inicializa la conexión serial y retorna el objeto serial."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=timeout
        )
        print(f"Conectado al puerto {port} a {baud_rate} baudios.")
        time.sleep(2)  # Espera para que Arduino se inicialice
        return ser
    except serial.SerialException as e:
        print(f"Error al conectar al puerto {port}: {e}")
        return None

def send_rpm_command(ser, rpm1, rpm2, rpm3, rpm4):
    """Envía una cadena con los RPM de las 4 llantas en el formato M1:rpm1;M2:rpm2;M3:rpm3;M4:rpm4\n."""
    if ser is None or not ser.is_open:
        print("Error: No hay conexión serial activa.")
        return

    # Formatear la cadena
    command = f"M1:{rpm1};M2:{rpm2};M3:{rpm3};M4:{rpm4}\n"
    try:
        # Enviar la cadena codificada
        ser.write(command.encode('utf-8'))
        print(f"Enviado: {command.strip()}")
    except serial.SerialException as e:
        print(f"Error al enviar el comando: {e}")