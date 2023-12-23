import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
# from proximity_monitor.msg import ProximityWarning

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

def nearest_point_distance(ros_image):
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float64)
        min_distance = float(np.min(depth_array[depth_array!=0]))
        return min_distance * 0.001  # Convert to meters
    except CvBridgeError as e:
        print(e)

# ProximityWarning message: Float32MultiArray: [warning camera 1, proximity camera 1, warning camera 2, proximity camera 2]
# proximity: distance to the nearest point (0 to 1, where 1 is threshold meters away)
# warning: 1 if the distance is less than the threshold

class ProximityMonitorNode(Node):
    def __init__(self):
        super().__init__('proximity_monitor')

        # Declare parameters and initialize them
        self.declare_parameter('first_camera_name', 'camera1')
        self.declare_parameter('second_camera_name', 'camera2')
        self.declare_parameter('threshold', 0.2)

        # Get parameters
        self.first_camera_name = self.get_parameter('first_camera_name').get_parameter_value().string_value
        self.second_camera_name = self.get_parameter('second_camera_name').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        # Publishers and Subscribers
        self.warning_topic = 'proximity_warning'
        # self.second_warning_topic = 'second_proximity_warning'

        self.warning_publisher = self.create_publisher(Float32MultiArray, self.warning_topic, 10)
        # self.second_warning_publisher = self.create_publisher(Float32MultiArray, self.second_warning_topic, 10)

        self.first_camera_subscriber = self.create_subscription(Image, f'/{self.first_camera_name}/depth/image_rect_raw', self.first_camera_callback, 10)
        self.second_camera_subscriber = self.create_subscription(Image, f'/{self.second_camera_name}/depth/image_rect_raw', self.second_camera_callback, 10)

        self.get_logger().info(f'Listening for first camera on /{self.first_camera_name}/depth/image_rect_raw')
        self.get_logger().info(f'Listening for second camera on /{self.second_camera_name}/depth/image_rect_raw')

        self.first_camera_warning = None
        self.second_camera_warning = None
        self.first_camera_proximity = None
        self.second_camera_proximity = None

        self.timer = self.create_timer(0.1, self.publish_warning)

        # Dynamic reconfiguration callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def process_camera_data(self, data):
        proximity = nearest_point_distance(data)
        relative_proximity = proximity / self.threshold
        warning = 1.0 if proximity < self.threshold else 0.0
        return relative_proximity, warning

    def first_camera_callback(self, data):
        self.first_camera_proximity, self.first_camera_warning = self.process_camera_data(data)
        self.get_logger().debug(f'First camera: Proximity: {self.first_camera_proximity}, Warning: {self.first_camera_warning}')

    def second_camera_callback(self, data):
        self.second_camera_proximity, self.second_camera_warning = self.process_camera_data(data)
        self.get_logger().debug(f'Second camera: Proximity: {self.second_camera_proximity}, Warning: {self.second_camera_warning}')

    def publish_warning(self):
        warning_msg = Float32MultiArray()

        if self.first_camera_warning:
            self.get_logger().info(f'Publishing first camera warning: {self.first_camera_proximity}')
        if self.second_camera_warning:
            self.get_logger().info(f'Publishing second camera warning: {self.second_camera_proximity}')

        warning_msg.data = [
            self.first_camera_warning or float('NaN'),
            self.first_camera_proximity or float('NaN'),
            self.second_camera_warning or float('NaN'),
            self.second_camera_proximity or float('NaN')
        ]
        self.warning_publisher.publish(warning_msg)

        self.first_camera_warning = None
        self.second_camera_warning = None
        self.first_camera_proximity = None
        self.second_camera_proximity = None

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'threshold' and param.type_ == param.PARAMETER_DOUBLE:
                self.threshold = param.value
                self.get_logger().info(f'New threshold set: {self.threshold}')
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=False)

def main(args=None):
    rclpy.init(args=args)
    proximity_monitor_node = ProximityMonitorNode()
    proximity_monitor_node.get_logger().info('Proximity Monitor Node started')
    rclpy.spin(proximity_monitor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
