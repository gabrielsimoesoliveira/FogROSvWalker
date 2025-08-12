# Copyright 2022 The Regents of the University of California (Regents)
# ... (toda a licença e copyright permanecem os mesmos) ...
# IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

from launch_ros.actions import Node
import fogros2

# --- ADICIONE ESTAS IMPORTAÇÕES ---
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
# --- FIM DAS NOVAS IMPORTAÇÕES ---

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
        return "ami-036cafe742923b3d9"
        # Para Ubuntu 24.04 (Noble), você precisaria de um novo AMI ID aqui.
        # Exemplo (hipotético, verifique o AMI ID correto para sua região):
    # if ubuntu_release == "24.04":
    #     return "ami-xxxxxxxxxxxxxxxxx"


    raise ValueError(f"No AMI for {ubuntu_release}")


def generate_launch_description():
    """Talker example that launches the listener on AWS and loads params for talker.""" # Descrição atualizada
    ld = fogros2.FogROSLaunchDescription()
    machine1 = fogros2.AWSCloudInstance(
        region="us-west-1", ec2_instance_type="t2.xlarge", ami_image=ami_image()
    )

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )

    #talker_node = Node(
    talker_node = fogros2.CloudNode(
        package="fogros2_examples",
        executable="talker",
        output="screen",
        machine=machine1,
        parameters=[Path(get_package_share_directory('fogros2_examples'), 'param', 'talker_params.yaml')]
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
