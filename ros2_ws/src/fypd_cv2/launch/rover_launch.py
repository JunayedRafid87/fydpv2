from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'serial_port_lidar', default_value='/dev/ttyUSB0',
            description='Serial port for RPLiDAR C1'),
        DeclareLaunchArgument(
            'serial_port_esp32', default_value='/dev/ttyACM0',
            description='Serial port for ESP32-S3'),

        # ── 1. RPLiDAR C1 Driver (at 12 Hz scan rate) ──
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port_lidar'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
                'scan_frequency': 15.0,   # Set scan frequency to 15.0 Hz
            }],
            output='screen',
        ),

        # ── 2. Static TF: tilt_link → laser ──
        #    Old-style positional args for Humble: x y z yaw pitch roll frame_id child_frame_id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tilt_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'tilt_link', 'laser'],
        ),

        # ── 3. Static TF: base_link → thermal_camera_link ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.05', '0', '0.10', '0', '0', '0', 'base_link', 'thermal_camera_link'],
        ),

        # ── 4. Tilt Angle and Orientation TF Broadcaster ──
        Node(
            package='fypd_cv2',
            executable='tilt_tf_broadcaster',
            name='tilt_tf_broadcaster',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port_esp32'),
                'baud_rate': 115200,
                'parent_frame': 'map',
                'child_frame': 'base_link',
            }],
            output='screen',
        ),
    ])
