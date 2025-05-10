import threading
import time
import logging
import rclpy
import sensor_msgs
from rclpy.node import Node
from cv_bridge import CvBridge
from sensors.interfaces.MagProvider import MagProvider
from sensors.interfaces.camera_provider import CameraProvider
from sensors.interfaces.imu_provider import IMUProvider
from sensors.data_structures import CameraFrame, IMUData, MagData
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, Imu, MagneticField

class SimSensorProvider(CameraProvider, IMUProvider, MagProvider):
    def __init__(self, config):
        self.config = config
        self.latest_frame = None
        self.latest_imu = None
        self.latest_mag = None
        self.node = None
        self.initialized = False
        self.spin_thread = None
        self.fps = 0
        self.last_time = time.time()
        self.frame_count = 0
        self.ros_running = False

    def start(self):
        """
        Called by the parent code. This now just spawns a thread that sets up ROS
        and spins. The parent no longer blocks on rclpy.spin(...).
        """
        if not self.initialized:
            self.initialized = True
            self.spin_thread = threading.Thread(target=self._ros_thread_main, daemon=True)
            self.spin_thread.start()

    def _ros_thread_main(self):
        """
        All ROS-related work is done here in a separate thread.
        """
        rclpy.init(args=None)
        self.node = SimSensorSubscriber("sim_camera_subscriber", self)
        self.ros_running = True

        try:
            rclpy.spin(self.node)  # Blocking call in this thread only
        finally:
            # Cleanup after spin stops
            if self.node is not None:
                self.node.destroy_node()
            rclpy.shutdown()
            self.ros_running = False


    def stop(self):
        """
        Stop the ROS node. If spinning, this will end the spin so the thread can exit.
        """
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        # rclpy.shutdown() is called from inside the thread after spin ends.
        # If you want a clean shutdown, you can wait for the spin thread to exit:
        if self.spin_thread and self.spin_thread.is_alive():
            # This will force spin() to exit by shutting down rclpy.
            rclpy.shutdown()
            self.spin_thread.join()
        self.initialized = False

    def get_latest_camera_frame(self) -> CameraFrame:
        return self.latest_frame

    def get_latest_imu_data(self):
        return self.latest_imu

    def get_latest_mag_data(self):
        return self.latest_mag


class SimSensorSubscriber(Node):
    def __init__(self, node_name, camera_getter):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.sensor_getter = camera_getter

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            qos
        )
        self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            qos
        )

        self.create_subscription(
            MagneticField,
            'magnetometer',
            self.magnetometer_callback,
            qos
        )

        self.get_logger().info("SimCameraSubscriber started.")  # common start log
        self.last_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg : Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
        self.sensor_getter.latest_frame = CameraFrame(cv_image, time.time())
        self._log_fps(cv_image.shape[1], cv_image.shape[0])

    def imu_callback(self, msg: Imu):
        """New IMU callback to store and log readings."""
        orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        linear_acceleration = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        angular_velocity = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        self.sensor_getter.latest_imu = IMUData(orientation, linear_acceleration, angular_velocity, time.time())


    def magnetometer_callback(self, msg: MagneticField):
        mag_data = MagData(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)
        self.sensor_getter.latest_mag = mag_data


    def _log_fps(self, width, height):
        current_time = time.time()
        self.frame_count += 1
        elapsed = current_time - self.last_time
        if elapsed >= 1.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"Sim Camera FPS: {fps:.2f}, Resolution: {width}x{height}")
            self.last_time = current_time
            self.frame_count = 0
