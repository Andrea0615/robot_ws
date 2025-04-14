# -*- coding: utf-8 -*-
import rospy
import numpy as np
import serial
from tu_paquete.msg import WheelInfo
from nav_msgs.msg import Odometry
import tf.transformations as tft
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

#!/usr/bin/env python

# Cola simple
class ListQueueSimple:
    def __init__(self):
        self._L = []
    def enqueue(self, item):
        self._L.append(item)
    def dequeue(self):
        if not self.isempty():
            return self._L.pop(0)
        return None
    def peek(self):
        if not self.isempty():
            return self._L[0]
        return None
    def __len__(self):
        return len(self._L)
    def isempty(self):
        return len(self) == 0
    

class VESCRPMListener: 
    def __init__(self):

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


rospy.init_node("odometry_node")

odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
wheelValues = rospy.Publisher("/wheel_setter", WheelInfo, queue_size=10)

rate = rospy.Rate(20)  # 20 Hz

# Calcula el ángulo de Ackermann ajustado para evitar que el robot gire demasiado
def anguloAckermann(delta, v2):
    distancia1 = np.tan(delta) * 28.15
    d2 = distancia1 + 89
    Beta = np.atan(28.15 / d2)
    b = (np.pi / 2) - Beta
    a = (np.pi/2) - delta
    v1 = ((np.tan(a)*28.15)/(89+(np.tan(a)*28.15)))*v2
    return b, v1

#Queue dinamica para agregar piedras como waypoints
def generar_ruta_prioritaria(piedras_lista):
    piedras = piedras_lista
    puntos_establecidos = ListQueueSimple()
    waypoints = ListQueueSimple()

    # Waypoints originales
    waypoints_array = np.array([
        [1.295, 1.5], [1.295, 6.5], [3.777, 6.5],
        [3.777, 1.5], [6.475, 1.5], [6.475, 6.5],
        [9.065, 6.5], [9.065, 1.5], [1.295, 1.5]
    ])
    for punto in waypoints_array:
        puntos_establecidos.enqueue(punto)

    # Bucle principal de decisión de waypoint
    while not puntos_establecidos.isempty() or not piedras.isempty():
        if not piedras.isempty():
            punto = piedras.dequeue()
            print("Tomando prioridad: piedra detectada en", punto)
        elif not puntos_establecidos.isempty():
            punto = puntos_establecidos.dequeue()
            print("Siguiendo ruta establecida hacia", punto)

        waypoints.enqueue(punto)
    
    return waypoints


# Encuentra el punto objetivo ("look-ahead") en la trayectoria
def findLookAheadPoint(x, y, waypoints, idxCurrent, Ld):
    N = waypoints.shape[0]
    lx = waypoints[-1, 0]
    ly = waypoints[-1, 1]
    idxNext = idxCurrent
    robotPos = np.array([x, y])

    # Itera sobre los segmentos para encontrar el punto de seguimiento
    while idxNext < N - 1:
        segStart = waypoints[idxNext]
        segEnd = waypoints[idxNext + 1]
        segVec = segEnd - segStart
        segLen = np.linalg.norm(segVec)
        toStart = robotPos - segStart
        proj = np.dot(toStart, segVec) / segLen
        remain = segLen - proj

        if remain < 0:
            idxNext += 1
            continue

        s = Ld if remain >= Ld else remain
        param = proj + s
        paramFrac = max(0, min(1, param / segLen))
        lookPt = segStart + paramFrac * segVec
        lx = lookPt[0]
        ly = lookPt[1]

        if s >= remain:
            idxNext += 1
        else:
            break

    return lx, ly, idxNext

# Calcula la matriz Jacobiana F para el filtro de Kalman extendido
def computeF(x, u, imu_data, L, dt):
    v = u[0]
    theta = x[2]
    
    a_x = imu_data['accel_filtered']['x']
    a_y = imu_data['accel_filtered']['y']
    
    F = np.array([
        [1, 0, -v * np.sin(theta) * dt, dt, 0, 0],
        [0, 1,  v * np.cos(theta) * dt, 0, dt, 0],
        [0, 0, 1,                 0, 0, dt],
        [0, 0, -a_x * dt * np.sin(theta), 1, 0, 0],
        [0, 0,  a_y * dt * np.cos(theta), 0, 1, 0],
        [0, 0, 0,                 0, 0, 1]
    ])
    return F

def f(x, u, imu_data, L, dt):
    v = u[0]
    delta = u[1]
    theta = x[2]
    
    a_x = imu_data['accel_filtered']['x']
    a_y = imu_data['accel_filtered']['y']
    omega = imu_data['gyro_filtered']['z']

    x_next = np.array([
        x[0] + x[3] * dt,
        x[1] + x[4] * dt,
        theta + omega * dt,
        v + a_x * dt * np.cos(theta),
        v + a_y * dt * np.sin(theta),
        omega
    ])
    return x_next

# Parámetros de simulación y del vehículo
dt = 0.05
maxTime = 120
lookAheadDist = 1.5
desiredSpeed = 0.4
realSpeed = realSpeedSetter()

L = 0.89 # distancia entre ejes
W = 0.563
wheelDiameter = 0.24
wheelCircumference = np.pi * wheelDiameter



# Trayectoria deseada (lista de waypoints)
waypoints = generar_ruta_prioritaria(piedras,ListQueueSimple)


# Estado inicial estimado
odom_x = 0.0
odom_y = 0.0
odom_theta = 0.0
odom_theta = np.radians(odom_theta)
xhat = np.array([odom_x, odom_y, odom_theta, 0.0, 0.0, 0.0])
P = np.identity(6) * 0.01 # Covarianza inicial
Q = np.diag([0.001, 0.001, 0.0005, 0.001, 0.001, 0.0001]) # Ruido del proceso
R = np.diag([0.02, 0.02, 0.01, 0.05, 0.005]) # Ruido de medición

traj_est = [[xhat[0], xhat[1]]]

odom_topic = {
    'pose': None,
    'velocity': None,
    'RPM': None
}

# Bucle principal de simulación
idxWaypoint = 0
t_total = 0

# Initialize the RPM listener
rpm_listener = VESCRPMListener()

while not rospy.is_shutdown():
    if idxWaypoint >= len(waypoints) - 1:
        print('Último waypoint alcanzado.')
        break
    # Determina el siguiente punto a seguir
    lookX, lookY, idxWaypoint = findLookAheadPoint(odom_x, odom_y, waypoints, idxWaypoint, lookAheadDist)

    dx = lookX - odom_x
    dy = lookY - odom_y
    L_d = np.hypot(dx, dy)
    alpha = np.arctan2(dy, dx) - odom_theta
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    kappa = 2 * np.sin(alpha) / max(L_d, 1e-9)
    delta = np.arctan(kappa * L)
    b, v_interior = anguloAckermann(delta, desiredSpeed)
    # Get latest RPM from the listener
    real_rpm = rpm_listener.rpm_value #Obtener los valores las rpm para la medición real
    real_velocity = (real_rpm / 60.0) * wheelCircumference #Transformar las rpm
 
    RPM = (desiredSpeed / wheelCircumference) * 60


    # Simulación de medición con ruido
    noise = np.random.normal(0, np.sqrt(np.diag(R)))
    z = np.array([
        odom_x,
        odom_y,
        odom_theta,
        real_velocity + imu_data['accel_filtered']['x'],
        imu_data['gyro_filtered']['z']
    ]) + noise

    # Odometría básica
    omega = (desiredSpeed / L) * np.tan(delta)
    odom_x += desiredSpeed * np.cos(odom_theta) * dt
    odom_y += desiredSpeed * np.sin(odom_theta) * dt
    odom_theta += omega * dt

    # Predicción del estado y la covarianza
    u = np.array([desiredSpeed, delta])
    xhat_pred = f(xhat, u, imu_data, L, dt)
    F = computeF(xhat, u, imu_data, L, dt)

    P_pred = np.dot(np.dot(F, P), F.T) + Q

    # Corrección (medición)
    H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 1]
    ])

    S = np.dot(np.dot(H, P_pred), H.T) + R

    K = np.dot(np.dot(P_pred, H.T), np.linalg.inv(S))

    xhat = xhat_pred + np.dot(K, (z - np.dot(H, xhat_pred)))

    P = np.dot(np.eye(6) - np.dot(K, H), P_pred)

    # Actualiza los datos para publicación simulada
    odom_topic['pose'] = np.array([xhat[0], xhat[1], xhat[2]])
    odom_topic['velocity'] = (xhat[3], xhat[5])
    odom_topic['RPM'] = RPM

    traj_est.append([xhat[0], xhat[1]])

    t_total += dt

    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.pose.pose.position.x = xhat[0]
    odom_msg.pose.pose.position.y = xhat[1]
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]


    quaternion = tft.quaternion_from_euler(0, 0, xhat[2])
    wheel_msg = WheelInfo()
    wheel_msg.v_interior = v_interior
    wheel_msg.desired_speed = desiredSpeed
    wheel_msg.delta = delta
    wheel_msg.beta = b
    
    odom_pub.publish(odom_msg)
    wheelValues.publish(wheel_msg)

    rate.sleep()