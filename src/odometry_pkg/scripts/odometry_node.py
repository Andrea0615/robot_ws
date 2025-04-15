#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf.transformations as tft
from tu_paquete.msg import WheelInfo

# Importación de módulos propios
from controller import angulo_ackermann, find_look_ahead_point
from efk import compute_F, predict_state
from utils import compute_quaternion, VESCRPMListener, IMUListener

# IMPORTAR LAS FUNCIONES DE CONEXIÓN SERIAL (asumiendo que están en serial_utils.py)
from utils import initialize_serial, send_rpm_command

# Parámetros de conexión serial (ajusta según tu sistema: 'COM6' para Windows o '/dev/ttyUSB0' para Linux/Mac)
SERIAL_PORT = 'COM6'
BAUD_RATE = 115200
TIMEOUT = 1

def main():
    rospy.init_node("odometry_node")
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    wheel_pub = rospy.Publisher("/wheel_setter", WheelInfo, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    dt = 0.05
    lookAheadDist = 1.5
    desiredSpeed = 0.4

    L = 0.89  # distancia entre ejes
    wheelDiameter = 0.24
    wheelCircumference = np.pi * wheelDiameter

    # Trayectoria deseada (lista de waypoints)
    waypoints = np.array([
        [1.295, 1.5], [1.295, 6.5], [3.777, 6.5],
        [3.777, 1.5], [6.475, 1.5], [6.475, 6.5],
        [9.065, 6.5], [9.065, 1.5], [1.295, 1.5]
    ])

    # Estado inicial estimado
    odom_x = 0.0
    odom_y = 0.0
    odom_theta = np.radians(0.0)
    xhat = np.array([odom_x, odom_y, odom_theta, 0.0, 0.0, 0.0])
    P = np.identity(6) * 0.01
    Q = np.diag([0.001, 0.001, 0.0005, 0.001, 0.001, 0.0001])
    R = np.diag([0.02, 0.02, 0.01, 0.05, 0.005])

    idxWaypoint = 0
    t_total = 0

    rpm_listener = VESCRPMListener()
    imu_listener = IMUListener()

    # Inicializar la conexión serial para enviar comandos de RPM
    ser = initialize_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)

    while not rospy.is_shutdown():
        if idxWaypoint >= len(waypoints) - 1:
            rospy.loginfo("Último waypoint alcanzado.")
            break

        # Cálculo del punto de seguimiento
        lookX, lookY, idxWaypoint = find_look_ahead_point(odom_x, odom_y, waypoints, idxWaypoint, lookAheadDist)

        dx = lookX - odom_x
        dy = lookY - odom_y
        L_d = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - odom_theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        kappa = 2 * np.sin(alpha) / max(L_d, 1e-9)
        delta = np.arctan(kappa * L)
        
        # Función que retorna dos valores: 'b' y 'v_interior'
        b, v_interior = angulo_ackermann(delta, desiredSpeed)
        
        # Cálculo de RPM basados en la circunferencia de la llanta
        rpm_exterior = (desiredSpeed / wheelCircumference) * 60
        rpm_interior = (v_interior / wheelCircumference) * 60

        # Se obtiene la RPM leída del sensor (si se usa)
        real_RPM = rpm_listener.rpm_value  
        real_velocity = (real_RPM / 60.0) * wheelCircumference

        # Si lo que se desea es enviar el comando de RPM calculado, se puede elegir entre:
        #  - En línea recta, las 4 llantas con rpm_exterior
        #  - Al girar, se asignan las RPM de acuerdo al lado interior/exterior
        if delta > 0:
            # Giro a la izquierda: llantas izquierdas interiores
            rpm_left = rpm_interior
            rpm_right = rpm_exterior
        elif delta < 0:
            # Giro a la derecha: llantas derechas interiores
            rpm_left = rpm_exterior
            rpm_right = rpm_interior
        else:
            rpm_left = rpm_right = rpm_exterior

        # Enviar comando de RPM a las llantas (se asume:
        #   M1 y M3: llantas izquierdas,
        #   M2 y M4: llantas derechas)
        send_rpm_command(ser, rpm_left, rpm_right, rpm_left, rpm_right)

        # Simulación de medición con ruido
        noise = np.random.normal(0, np.sqrt(np.diag(R)))
        z = np.array([
            odom_x,
            odom_y,
            odom_theta,
            real_velocity + imu_listener['accel_filtered']['x'],
            imu_listener['gyro_filtered']['z']
        ]) + noise

        # Actualización de la odometría básica
        omega = (real_velocity / L) * np.tan(delta)
        odom_x += real_velocity * np.cos(odom_theta) * dt
        odom_y += real_velocity * np.sin(odom_theta) * dt
        odom_theta += omega * dt

        # Predicción y corrección usando EKF
        u = np.array([real_velocity, delta])
        xhat_pred = predict_state(xhat, u, L, dt)
        F = compute_F(xhat, u, dt)
        P_pred = F @ P @ F.T + Q

        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        xhat = xhat_pred + K @ (z - H @ xhat_pred)
        P = (np.eye(6) - K @ H) @ P_pred

        t_total += dt

        # Preparar y publicar el mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = xhat[0]
        odom_msg.pose.pose.position.y = xhat[1]
        odom_msg.pose.pose.position.z = 0

        quaternion = compute_quaternion(xhat[2])
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Preparar y publicar el mensaje de los datos de las ruedas
        wheel_msg = WheelInfo()
        wheel_msg.v_interior = v_interior
        wheel_msg.desired_speed = desiredSpeed
        wheel_msg.delta = delta
        wheel_msg.beta = b

        odom_pub.publish(odom_msg)
        wheel_pub.publish(wheel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass