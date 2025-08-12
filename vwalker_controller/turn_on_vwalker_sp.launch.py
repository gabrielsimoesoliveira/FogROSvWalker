import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import fogros2 
from pathlib import Path
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def ami_image():
    # An AMI is an Amazon Web Services virtual machine image with a
    # pre-installed OS and dependencies.  We match the AMI in the
    # cloud to have the same OS release as the robot.  Currently we
    # support Ubuntu 20.04 and 22.04.

    import lsb_release

    ubuntu_release = lsb_release.get_os_release()["RELEASE"]

    if ubuntu_release == "20.04":
        return "ami-00f25057ddc9b310b"
    if ubuntu_release == "22.04":
        # "ami-034160df82745c454" is custom AMI
        return "ami-03cae36d8ec62f343"


    raise ValueError(f"No AMI for {ubuntu_release}")
    

def generate_launch_description():

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='true')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    hri_1_node_params = {
        'assymetric_force': False
    }
    
    hri_2_node_params = {
        'mv': 0.2,
        'mw': 0.1,
        'dv': 90.0,
        'dw': 38.0
    }
    
    hri_3_node_params = {
        'user/weight': 30.0, 
        'use_distance': False,
        'max_distance': 1.2
    }
    
    ld = fogros2.FogROSLaunchDescription()     # substitui o ld = LaunchDescription()
    
    # inicialização de máquina virtual em são paulo, brasil
    machine1 = fogros2.AWSCloudInstance(
        region="sa-east-1", ec2_instance_type="t2.xlarge", ami_image=ami_image()
    )

    motor_controller_node = Node(
        package='vwalker_controller',
        executable='motor_controller_node',
        name='motor_controller',
        output='screen',
        parameters=[Path(get_package_share_directory('vwalker_controller'), 'param', 'motor_controller_node.yaml')]
    )

    force_acquisition_node = Node(
            package='vwalker_force',
            executable='force_acquisition_node',
            name='force_acquisition',
            output='screen',
            parameters=[Path(get_package_share_directory('vwalker_force'), 'param', 'force_acquisition_node.yaml')]
        )

    #force filter node inicializado na nuvem
    hri_1_node = fogros2.CloudNode(
        package='vwalker_hri',
        executable='force_filter_node',
        name='force_filter', 
        output='screen',
        machine=machine1,
        parameters=[hri_1_node_params] # Parâmetros inline
    )

    #admittance controller inicializado na nuvem
    hri_2_node = fogros2.CloudNode(
        package='vwalker_hri',
        executable='admittance_controller_node',
        name='admittance_controller',
        output='screen',
        machine=machine1,
        parameters=[hri_2_node_params] # Parâmetros inline
    )

    hri_3_node = Node(
        package='vwalker_hri',
        executable='superviser_node',
        name='superviser',
        output='screen',
        parameters=[hri_3_node_params] # Parâmetros inline
    )
    
    # SLLIDAR NODE
    DeclareLaunchArgument(
        'channel_type',
        default_value=channel_type,
        description='Specifying channel type of lidar'),
        
    DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar'),

    DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar'),
    
    DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar'),

    DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data'),

    DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data'),
    
    DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar'),
    
    
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':channel_type,
                    'serial_port': serial_port, 
                    'serial_baudrate': serial_baudrate, 
                    'frame_id': frame_id,
                    'inverted': inverted,
                    'angle_compensate': angle_compensate}],
        output='screen')

    leg_monitoring_node = Node(
        package='leg_monitoring',
        executable='leg_laser',
        name='leg_laser',
        output='screen' ,
        parameters=[Path(get_package_share_directory('leg_monitoring'), 'param', 'leg_monitoring.yaml')]
    )
    
    vwalker_unity_node = Node(
        package='vwalker_unity',
        executable='scan_renderer_vr',
        name='scan_unity_node',
        output='screen',
    )

    vwalker_imu_fusion_node = Node(
        package='vwalker_imu_fusion',
        executable='imu_fusion',
        name='imu_fusion_node',
        output='screen',
    )
    
    
    lidar_broadcaster = Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='lidar_broadcaster',
            arguments=['0.410', '0', '0.420', '0', '0', '0', '1','base_link','laser'],
    )

    imu_broadcaster = Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='imu_broadcaster',
            arguments=['0.560', '0', '0.280', '0', '0', '0', '1','base_link','imu_link'],
    )

    ld.add_action(motor_controller_node)
    ld.add_action(force_acquisition_node)
    ld.add_action(hri_1_node)
    ld.add_action(hri_2_node)
    ld.add_action(hri_3_node)
    ld.add_action(sllidar_node)
    ld.add_action(leg_monitoring_node)
    ld.add_action(vwalker_unity_node)
    ld.add_action(vwalker_imu_fusion_node)
    ld.add_action(lidar_broadcaster)
    ld.add_action(imu_broadcaster)


    return ld
