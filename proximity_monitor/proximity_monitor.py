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
        return min_distance
    except CvBridgeError as e:
        print(e)

# ProximityWarning message: Float32MultiArray: [warning, proximity]
# proximity: distance to the nearest point (0 to 1, where 1 is threshold meters away)
# warning: 1 if the distance is less than the threshold

class ProximityMonitorNode(Node):
    def __init__(self):
        super().__init__('proximity_monitor')

        # Declare parameters and initialize them
        self.declare_parameter('first_camera_name', 'camera1')
        self.declare_parameter('second_camera_name', 'camera2')
        self.declare_parameter('threshold', 1.0)

        # Get parameters
        self.first_camera_name = self.get_parameter('first_camera_name').get_parameter_value().string_value
        self.second_camera_name = self.get_parameter('second_camera_name').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        # Publishers and Subscribers
        self.first_warning_topic = 'first_proximity_warning'
        self.second_warning_topic = 'second_proximity_warning'

        self.first_warning_publisher = self.create_publisher(Float32MultiArray, self.first_warning_publisher, 10)
        self.second_warning_publisher = self.create_publisher(Float32MultiArray, self.second_warning_topic, 10)

        self.first_camera_subscriber = self.create_subscription(Image, f'{self.first_camera_name}/depth', self.first_camera_callback, 10)
        self.second_camera_subscriber = self.create_subscription(Image, f'{self.second_camera_name}/depth', self.second_camera_callback, 10)

        # Dynamic reconfiguration callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def first_camera_callback(self, data):
        proximity, warning = self.process_camera_data(data)
        proximity_warning = Float32MultiArray()
        proximity_warning.data = [proximity, warning]
        self.first_warning_publisher.publish(warning)

    def second_camera_callback(self, data):
        proximity, warning = self.process_camera_data(data)
        proximity_warning = Float32MultiArray()
        proximity_warning.data = [proximity, warning]
        self.second_warning_publisher.publish(warning)

    def process_camera_data(self, data):
        proximity = nearest_point_distance(data)
        warning = proximity < self.threshold
        return proximity, warning

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
    rclpy.spin(proximity_monitor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
