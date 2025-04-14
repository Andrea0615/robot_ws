# utils.py
import numpy as np
import rospy
import tf.transformations as tft
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from threading import Lock

class ListQueueSimple:
    """
    A thread-safe queue for storing items (e.g., 2D coordinates) with FIFO behavior.
    Supports enqueue, dequeue, isempty, and push_front for prioritization.
    """
    def __init__(self):
        self.items = []
        self.lock = Lock()

    def enqueue(self, item):
        """Add an item to the tail of the queue."""
        with self.lock:
            self.items.append(item)

    def push_front(self, item):
        """Add an item to the head of the queue (for prioritization)."""
        with self.lock:
            self.items.insert(0, item)

    def dequeue(self):
        """Remove and return an item from the head of the queue."""
        with self.lock:
            if self.items:
                return self.items.pop(0)
            return None

    def isempty(self):
        """Check if the queue is empty."""
        with self.lock:
            return len(self.items) == 0

    def size(self):
        """Return the number of items in the queue."""
        with self.lock:
            return len(self.items)

class IMUListener:
    def _init_(self):
        # Initialize storage for each IMU value
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accel_filtered = {'x': 0.0, 'y': 0.0, 'z': 0.0}
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
  # Assume this is your queue implementation

class CoordinatesListener:
    def __init__(self):
        self.rock_coords = ListQueueSimple()
        rospy.Subscriber("obstacle_coordinates", Point, self.callback)

    def callback(self, msg):
        coord = [msg.x, msg.y]  # Use list for compatibility with waypoints
        self.rock_coords.enqueue(coord)  # Or push_front(coord) for newest-first priority

    def get_new_coords(self):
        return self.rock_coords