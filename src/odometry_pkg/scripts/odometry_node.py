# odometry_node.py
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf.transformations as tft
from tu_paquete.msg import WheelInfo
import tf

# Importación de módulos propios
from controller import angulo_ackermann, find_look_ahead_point
from efk import compute_F, predict_state
from utils import compute_quaternion, VESCRPMListener, IMUListener

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

    # Trayectoria deseada (lista de waypoints) / Volver a sacar
    waypoints = np.array([
        [1.295, 1.5], [1.295, 6.5], [3.777, 6.5],
        [3.777, 1.5], [6.475, 1.5], [6.475, 6.5],
        [9.065, 6.5], [9.065, 1.5], [1.295, 1.5]
    ])

    # Estado inicial estimado
    odom_x = 0.0
    odom_y = 0.0
    odom_theta = np.radians(0.0)
    xhat = np.array([odom_x, odom_y, odom_theta, 0.0, 0.0, 0.0]) #np?
    P = np.identity(6) * 0.01
    Q = np.diag([0.001, 0.001, 0.0005, 0.001, 0.001, 0.0001]) #X que valores extra en especifico?
    R = np.diag([0.02, 0.02, 0.01, 0.05, 0.005])

    idxWaypoint = 0
    t_total = 0

    rpm_listener = VESCRPMListener()
    imu_listener = IMUListener()

    tf_broadcaster = tf.TransformBroadcaster() #Crea un objeto que se encarga de publicar transformaciones (Le agregue)

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
        b, v_interior = angulo_ackermann(delta, desiredSpeed)
        real_RPM = rpm_listener.rpm_value
        real_velocity = (real_RPM/ 60.0) * wheelCircumference

        RPM = (desiredSpeed / wheelCircumference) * 60

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
        u = np.array([real_velocity, delta]) #Preguntarle a DIOS
        xhat_pred = predict_state(xhat, u, L, dt)
        F = compute_F(xhat, u, dt)
        P_pred = F @ P @ F.T + Q

        # Matriz de observación
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
        odom_msg.header.frame_id = "odom" #Referencia global
        odom_msg.child_frame_id = "base_link" #Frame del robot
        odom_msg.pose.pose.position.x = xhat[0]
        odom_msg.pose.pose.position.y = xhat[1]
        odom_msg.pose.pose.position.z = 0

        # Calcular el cuaternión antes de asignarlo
        quaternion = compute_quaternion(xhat[2])
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        #Publicar odom
        odom_pub.publish(odom_msg)

        #Publicar la transformacion t entre odom y base_link
        tf_broadcaster.sendTransform(
            (xhat[0], xhat[1], 0),
            quaternion,
            rospy.Time.now(),
            "base_link", 
            "odom"
        )

        # Preparar y publicar el mensaje de los datos de las ruedas
        wheel_msg = WheelInfo()
        wheel_msg.v_interior = v_interior
        wheel_msg.desired_speed = desiredSpeed
        wheel_msg.delta = delta
        wheel_msg.beta = b

        wheel_pub.publish(wheel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass