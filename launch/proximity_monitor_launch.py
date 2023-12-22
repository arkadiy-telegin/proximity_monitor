from launch import LaunchDescription
from launch.actions import Node, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

configurable_parameters = [
    {'name': 'first_camera_name', 'default': 'camera1', 'description': 'name of the first camera'},
    {'name': 'second_camera_name', 'default': 'camera2', 'description': 'name of the second camera'},
    {'name': 'threshold', 'default': 1.0, 'description': 'distance threshold for proximity warning in meters'},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
        Node(package='proximity_monitor',
             executable='proximity_monitor',
             name='proximity_monitor',
             parameters=[set_configurable_parameters(configurable_parameters)],
        ),
    ])