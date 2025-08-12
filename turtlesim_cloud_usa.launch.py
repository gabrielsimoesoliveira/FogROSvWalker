# ~/fog_ws/src/my_turtlesim_control/launch/turtlesim_cloud_usa.launch.py
# nó de controle rodando na vm AWS no Norte da Califórnia, EUA

import fogros2
from launch import LaunchDescription
from launch_ros.actions import Node

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
        return "ami-036cafe742923b3d9"

    raise ValueError(f"No AMI for {ubuntu_release}")
    


def generate_launch_description():
    
    ld = fogros2.FogROSLaunchDescription() # substitui o comando ld = LaunchDescription()
    
    aws_machine = fogros2.AWSCloudInstance(
        region="us-west-1",  # Região AWS escolhida: Norte da Califórnia, EUA
        ec2_instance_type="t2.xlarge", # Tipo de instância
        ami_image=ami_image()
    )

    # nó do turtlesim (visualização gráfica) rodando LOCALMENTE
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    # nó de controle rodando na NUVEM, como um CloudNode
    automatic_control_node = fogros2.CloudNode(
        package="my_turtlesim_control",
        executable="automatic_control_node",
        output="screen",
        machine=aws_machine 
    )

    ld.add_action(turtlesim_node)
    ld.add_action(automatic_control_node)
    
    return ld
