# speed_config.py
# speed_controllers.py
# speed_guard_node.py
# vision_speed_guard  (package)


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray, LabelInfo
from ackermann_msgs.msg import AckermannDriveStamped

from ros2_pydata import from_detection2d_array, from_label_info
from .speed_config import controller 

class PerceptionSpeedGuard(Node):
    """
    ROS node that reads detections and publishes a speed limit.

    All sign logic lives in DriveController (speed_targets.py).
    This node only handles ROS plumbing: subscribe → convert → publish.
    """

    def __init__(self):
        super().__init__('perception_speed_guard')
        self.seq          = 0
        self.speed_limit  = 0.0
        self.controller   = controller
        self.id2label     = {}

        # Sensor QoS (keep only the latest message)
        qos = qos_profile_sensor_data
        qos.depth = 1

        # Latched QoS for label mapping (delivered to late subscribers too)
        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(LabelInfo, '/label_mapping', self.label_cb, qos_latched)
        self.create_subscription(Detection2DArray, '/detections_2d', self.detection_cb, qos)

        self.speed_pub = self.create_publisher(Float32, '/speed_limit', qos)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def label_cb(self, msg: LabelInfo):
        """Store label↔id mapping received from the detector node."""
        self.id2label = from_label_info(msg)
        self.get_logger().info(f'Label mapping received: {self.id2label}')

    def detection_cb(self, msg: Detection2DArray):
        """Convert detections, ask the controller for a speed, publish it."""
        detections = from_detection2d_array(msg, self.seq)
        self.speed_limit = self.controller.update(detections)

        speed_msg = Float32(data=self.speed_limit)
        self.speed_pub.publish(speed_msg)

        self.get_logger().info(
            f'speed={self.speed_limit:.2f}  '
            f'(regulatory={self.controller.speed_limit:.2f}, '
            f'drive={self.controller.drive_limit:.2f})'
        )
        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSpeedGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()