from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ✅ Launch Arguments 설정 (IMU 토픽 & use_sim_time)
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_x = LaunchConfiguration('base_x')
    base_y = LaunchConfiguration('base_y')
    base_z = LaunchConfiguration('base_z')
    return LaunchDescription([
        # ✅ IMU 토픽을 설정할 수 있도록 설정 (기본값: "/robot0/imu")
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/robot0/imu',
            description='IMU topic name for TF broadcaster'
        ),

        # ✅ 시뮬레이션 사용 여부 설정 (기본값: false)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (Gazebo or other simulators)'
        ),

        DeclareLaunchArgument('base_x', default_value='0.0', description='Base link X offset'),
        DeclareLaunchArgument('base_y', default_value='0.0', description='Base link Y offset'),
        DeclareLaunchArgument('base_z', default_value='0.36288', description='Base link Z offset'),

        # ✅ `slam_tf_broadcaster` 실행 시 `use_sim_time`과 `imu_topic` 전달
        Node(
            package='slam_tf_broadcaster',
            executable='slam_tf_broadcaster',
            name='slam_tf_broadcaster',
            output='screen',
            parameters=[{
                "imu_topic": imu_topic,
                "use_sim_time": use_sim_time,  # ✅ 추가됨
                "base_x": base_x,
                "base_y": base_y,
                "base_z": base_z
            }]
        )
    ])
