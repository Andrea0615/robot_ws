import rospy
import numpy as np
from nav_msgs.msg import Odometry #Cambiar después
from odometry.msg import WheelInfo #Cambiar después
from controller import angulo_ackermann, find_look_ahead_point, generar_ruta_prioritaria
from efk import compute_F, predict_state
from utils import get_imu_data, compute_quaternion, VESCRPMListener, IMUListener, CoordinatesListener

def main():
    rospy.init_node("odometry_node")
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    wheel_pub = rospy.Publisher("/wheel_setter", WheelInfo, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    dt = 0.05
    lookAheadDist = 1.5
    desiredSpeed = 0.4
    L = 0.89
    wheelDiameter = 0.24
    wheelCircumference = np.pi * wheelDiameter

    # Estado inicial
    odom_x = 0.0
    odom_y = 0.0
    odom_theta = np.radians(0.0)
    xhat = np.array([odom_x, odom_y, odom_theta, 0.0, 0.0, 0.0])
    P = np.identity(6) * 0.01
    Q = np.diag([0.001, 0.001, 0.0005, 0.001, 0.001, 0.0001])
    R = np.diag([0.02, 0.02, 0.01, 0.05, 0.005])

    idxWaypoint = 0
    waypoints = []  # Dynamic waypoint list
    rpm_listener = VESCRPMListener()
    imu_listener = IMUListener()
    coordenadas_camara = CoordinatesListener()

    while not rospy.is_shutdown():
        # Get next waypoint
        piedras = coordenadas_camara.get_new_coords()
        next_point = generar_ruta_prioritaria(piedras, use_push_front=False)
        
        if next_point is None and not waypoints:
            rospy.loginfo("No more waypoints or stones.")
            break

        if next_point:
            waypoints.append(next_point)
            # Keep waypoints list manageable (e.g., remove old points)
            if len(waypoints) > 100:
                waypoints = waypoints[-50:]

        if not waypoints:
            continue

        # Convert to NumPy array for find_look_ahead_point
        waypoints_array = np.array(waypoints)
        
        # Compute look-ahead point
        lookX, lookY, idxWaypoint = find_look_ahead_point(
            odom_x, odom_y, waypoints_array, idxWaypoint, lookAheadDist
        )

        # Remove waypoints behind the robot
        if idxWaypoint > 0:
            waypoints = waypoints[idxWaypoint:]
            idxWaypoint = 0

        # Control calculations
        dx = lookX - odom_x
        dy = lookY - odom_y
        L_d = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - odom_theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        kappa = 2 * np.sin(alpha) / max(L_d, 1e-9)
        delta = np.arctan(kappa * L)
        b, v_interior = angulo_ackermann(delta, desiredSpeed)

        # Sensor data
        real_RPM = rpm_listener.rpm_value
        real_velocity = (real_RPM / 60.0) * wheelCircumference
        RPM = (desiredSpeed / wheelCircumference) * 60
        imu_data = get_imu_data()

        # Simulated measurement with noise
        noise = np.random.normal(0, np.sqrt(np.diag(R)))
        z = np.array([
            odom_x,
            odom_y,
            odom_theta,
            real_velocity + imu_data['accel_filtered']['x'],
            imu_data['gyro_filtered']['z']
        ]) + noise

        # Update odometry
        omega = (real_velocity / L) * np.tan(delta)
        odom_x += real_velocity * np.cos(odom_theta) * dt
        odom_y += real_velocity * np.sin(odom_theta) * dt
        odom_theta += omega * dt

        # EKF update
        u = np.array([real_velocity, delta])
        xhat_pred = predict_state(xhat, u, imu_data, L, dt)
        F = compute_F(xhat, u, imu_data, dt)
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

        # Publish messages
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