# utils.py
import numpy as np
import rospy
import tf.transformations as tft
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from threading import Lock
import time 

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
    def __init__(self, sync_obj):
        self.sync = sync_obj
        self.imu_data = {
            'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }

        rospy.Subscriber("imu/yaw", Float32, self.yaw_callback)
        rospy.Subscriber("imu/pitch", Float32, self.pitch_callback)
        rospy.Subscriber("imu/roll", Float32, self.roll_callback)

        rospy.Subscriber("imu/accel_x", Float32, self.ax_callback)
        rospy.Subscriber("imu/accel_y", Float32, self.ay_callback)
        rospy.Subscriber("imu/accel_z", Float32, self.az_callback)

        rospy.Subscriber("imu/gyro_x", Float32, self.gx_callback)
        rospy.Subscriber("imu/gyro_y", Float32, self.gy_callback)
        rospy.Subscriber("imu/gyro_z", Float32, self.gz_callback)

    def yaw_callback(self, msg): self.imu_data['yaw'] = msg.data; self.sync.update_imu(self.imu_data.copy())
    def pitch_callback(self, msg): self.imu_data['pitch'] = msg.data
    def roll_callback(self, msg): self.imu_data['roll'] = msg.data

    def ax_callback(self, msg): self.imu_data['accel']['x'] = msg.data
    def ay_callback(self, msg): self.imu_data['accel']['y'] = msg.data
    def az_callback(self, msg): self.imu_data['accel']['z'] = msg.data

    def gx_callback(self, msg): self.imu_data['gyro']['x'] = msg.data
    def gy_callback(self, msg): self.imu_data['gyro']['y'] = msg.data
    def gz_callback(self, msg): self.imu_data['gyro']['z'] = msg.data


def compute_quaternion(theta):
    return tft.quaternion_from_euler(0, 0, theta)

class VESCRPMListener:
    def __init__(self, sync_obj):
        self.sync = sync_obj
        rospy.Subscriber("vesc/rpm", Int32, self.rpm_callback)

    def rpm_callback(self, msg):
        self.sync.update_rpm(msg.data)


class CoordinatesListener:
    def __init__(self):
        self.rock_coords = ListQueueSimple()
        rospy.Subscriber("obstacle_coordinates", Point, self.callback)

    def callback(self, msg):
        coord = [msg.x, msg.y]  # Use list for compatibility with waypoints
        self.rock_coords.enqueue(coord)  # Or push_front(coord) for newest-first priority

    def get_new_coords(self):
        return self.rock_coords

class SynchronizedData:
    def __init__(self):
        self.lock = Lock()
        self.last_update_time = 0
        self.imu_data = {}
        self.rpm = 0

    def update_imu(self, imu_dict):
        with self.lock:
            self.imu_data = imu_dict
            self.last_update_time = time.time()

    def update_rpm(self, rpm_value):
        with self.lock:
            self.rpm = rpm_value
            self.last_update_time = time.time()

    def get_latest_data(self):
        with self.lock:
            return {
                'imu': self.imu_data,
                'rpm': self.rpm,
                'timestamp': self.last_update_time
            }
