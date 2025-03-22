import cv2
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from camera.camera_getter.base_camera_getter import BaseCameraGetter
from cv_bridge import CvBridge


class SimCameraGetter(BaseCameraGetter):
    def __init__(self, config):
        self.config = config
        self.latest_frame = None
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
        self.node = SimCameraSubscriber("sim_camera_subscriber", self)
        self.ros_running = True

        try:
            rclpy.spin(self.node)  # Blocking call in this thread only
        finally:
            # Cleanup after spin stops
            if self.node is not None:
                self.node.destroy_node()
            rclpy.shutdown()
            self.ros_running = False

    def get_frame(self):
        """
        The main thread or parent code can call this to get the most recent frame.
        """
        return self.latest_frame

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
            # This will force spin() to exit by shutting down rclpy (or just let the node be destroyed).
            rclpy.shutdown()
            self.spin_thread.join()
        cv2.destroyAllWindows()
        self.initialized = False


class SimCameraSubscriber(Node):
    def __init__(self, node_name, camera_getter):
        super().__init__(node_name)

        self.bridge = CvBridge()
        self.camera_getter = camera_getter
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            qos_profile
        )

        self.get_logger().info("SimCameraGetter started.")
        self.last_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg):
        # Convert ROS image to CV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Store the frame in the camera_getter
        self.camera_getter.latest_frame = cv_image

        # Calculate FPS + resolution
        current_time = time.time()
        self.frame_count += 1
        elapsed = current_time - self.last_time
        if elapsed >= 1.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"Sim Camera FPS: {fps:.2f}, Resolution: {cv_image.shape[1]}x{cv_image.shape[0]}")
            self.last_time = current_time
            self.frame_count = 0


