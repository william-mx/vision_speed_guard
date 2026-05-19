import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy
from ament_index_python.packages import get_package_prefix

from vision_msgs.msg import Detection2DArray, LabelInfo
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

from ros2_pydata import from_detection2d_array, from_label_info

from perception_speed_policy.speed_controller import SpeedController
from perception_speed_policy.helpers import plot_speed_history, to_float

class PerceptionSpeedGuard(Node):
    def __init__(self):
        super().__init__('perception_speed_guard')

        # QoS for sensor data
        self.qos_profile = qos_profile_sensor_data
        self.qos_profile.depth = 1

        # QoS for label mapping (transient local — latched)
        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Optional label mapping subscription
        self.label_sub = self.create_subscription(
            LabelInfo,
            '/label_mapping',
            self.label_mapping_callback,
            qos_transient,
        )

        # RC Ackermann command subscription
        self.rc_ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            '/rc/ackermann_cmd',
            self.rc_ackermann_callback,
            self.qos_profile,
        )

        # Detection subscription
        self.det_sub = self.create_subscription(
            Detection2DArray,
            '/detections_2d',
            self.detection_callback,
            self.qos_profile,
        )

        # Speed limit publisher
        self.speed_limit_pub = self.create_publisher(
            Float32,
            '/speed_limit',
            self.qos_profile,
        )

        self.speed_ctrl = SpeedController()
        self.speed_history = []
        self.gt_speed = 0.0

    def rc_ackermann_callback(self, msg: AckermannDriveStamped):
        self.gt_speed = msg.drive.speed  # Store ground-truth speed for later plotting/evaluation

    def label_mapping_callback(self, msg: LabelInfo):
        self.id2label = from_label_info(msg)
        self.label2id = {lbl: idx for idx, lbl in self.id2label.items()}
        self.get_logger().info(f"Label mapping received: {self.id2label}")

    def detection_callback(self, msg: Detection2DArray):

        # Convert ROS detections into a simpler internal format
        all_detections = from_detection2d_array(msg)

        # Update the speed controller using current detections
        speed_limit = self.speed_ctrl.update(all_detections)

        # Store predictions and ground truth for later evaluation
        self.speed_history.append([speed_limit, self.gt_speed])

        # Publish the selected speed limit
        self.speed_limit_pub.publish(to_float(speed_limit))

    def plot_results(self, pkg_path):
        plots_path = pkg_path + '/plots/speed_profile.png'
        plot_speed_history(self.speed_history, plots_path)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSpeedGuard()

    pkg_path = get_package_prefix('perception_speed_policy').replace('install', 'src')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results(pkg_path)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()