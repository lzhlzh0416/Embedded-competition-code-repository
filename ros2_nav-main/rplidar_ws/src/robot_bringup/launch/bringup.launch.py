import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():
    robot_bringup_dir = get_package_share_directory(
        'robot_bringup')
    rplidar_ros2_dir = get_package_share_directory(
        'rplidar_ros')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    odom2tf = launch_ros.actions.Node(
        package='robot_bringup',
        executable='odom2tf',
        output='screen'
    )

    can_bridge = launch_ros.actions.Node(
        package='can_bridge_cpp',
        executable='can_sender_cpp',
        output= 'screen'

    )

    rplidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [rplidar_ros2_dir, '/launch', '/rplidar_a1_launch.py']),
    )

    # 使用 TimerAction 启动后 5 秒执行 ydlidar 节点
    ydlidar_delay = launch.actions.TimerAction(period=5.0, actions=[rplidar])
    return launch.LaunchDescription([
       
        #open serial     
        ExecuteProcess(
            cmd=['sudo','chmod', '666', '/dev/ttyUSB0'],
            shell=False,
            output='screen'
        ),
        can_bridge,
        urdf2tf,
        odom2tf,
        ydlidar_delay
    ])