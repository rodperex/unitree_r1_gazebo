# Copyright 2026 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The Unitree R1 in Gazebo (Harmonic), driven as the real robot through unitree_sdk2:
#
#   Gazebo with a world and the R1 (model.simulation_urdf)
#   r1_sdk_sim          the firmware: the locomotion service, rt/arm_sdk, rt/lowstate, odometry
#   r1_state_publisher  /joint_states, /odom and the TF odom -> base_link, from the SDK DDS
#   robot_state_publisher  the TF of the robot (model.description_urdf)
#   a bridge            /clock and the head camera: /<name>/camera/image, depth_image, camera_info
#
#   ros2 launch unitree_r1_gazebo r1_gazebo.launch.py world:=house
#   ros2 launch unitree_r1_gazebo r1_gazebo.launch.py world:=/path/to/my_world.sdf headless:=true

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from unitree_r1_gazebo import model

PACKAGE = 'unitree_r1_gazebo'
WORLDS = ('empty', 'depot', 'house', 'hospital_room')


def world_file(world, headless):
  """The SDF of the world: a name of the package's worlds, or a file. The world is processed by
  xacro with the argument headless (a plain SDF is left as it is): Gazebo needs a file."""
  if world in WORLDS:
    world = os.path.join(get_package_share_directory(PACKAGE), 'worlds', f'{world}.sdf')
  if not os.path.isfile(world):
    raise RuntimeError(f'No world "{world}": one of {", ".join(WORLDS)}, or the path of an SDF')
  sdf = tempfile.mktemp(prefix='unitree_r1_gazebo_', suffix='.sdf')
  subprocess.run(['xacro', '-o', sdf, f'headless:={headless}', world], check=True)
  return sdf


def launch_setup(context):
  def arg(name):
    return LaunchConfiguration(name).perform(context)

  name = arg('name')
  headless = arg('headless').lower() in ('true', '1')
  gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
  camera = {f'/{name}/camera/image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
            f'/{name}/camera/depth_image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
            f'/{name}/camera/camera_info': ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')}
  sdk = ['--name', name, '--interface', arg('sdk_interface'), '--domain', arg('sdk_domain')]
  actions = [
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gz_sim),
      launch_arguments={'gz_args': f'-r -s {world_file(arg("world"), headless)}'}.items()),
    Node(package='ros_gz_sim', executable='create', output='screen',
         arguments=['-string', model.simulation_urdf(name=name), '-name', name,
                    '-x', arg('x'), '-y', arg('y'), '-z', '0', '-Y', arg('yaw')]),
    Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_gz_bridge',
         output='screen',
         arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'] +
                   [f'{topic}@{ros}[{gz}' for topic, (ros, gz) in camera.items()],
         parameters=[{'use_sim_time': True}]),
    Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
         parameters=[{'robot_description': model.description_urdf(), 'use_sim_time': True}]),
    Node(package=PACKAGE, executable='r1_sdk_sim', output='screen', arguments=sdk),
    Node(package=PACKAGE, executable='r1_state_publisher', output='screen',
         parameters=[{'use_sim_time': True, 'interface': arg('sdk_interface'),
                      'domain': int(arg('sdk_domain'))}]),
  ]
  if not headless:
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim),
                                            launch_arguments={'gz_args': '-g'}.items()))
  return actions


def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('world', default_value='empty',
                          description=f'{", ".join(WORLDS)}, or the path of a world (SDF, or an '
                                      'SDF xacro with an argument headless)'),
    DeclareLaunchArgument('headless', default_value='false',
                          description='true: no Gazebo window (the simulation runs the same)'),
    DeclareLaunchArgument('name', default_value='r1',
                          description='The model in Gazebo, and the prefix of its topics'),
    DeclareLaunchArgument('x', default_value='0.0', description='Where the robot starts (m)'),
    DeclareLaunchArgument('y', default_value='0.0', description='Where the robot starts (m)'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Its heading (rad)'),
    DeclareLaunchArgument('sdk_interface', default_value='lo',
                          description='The network interface of the DDS of unitree_sdk2'),
    DeclareLaunchArgument('sdk_domain', default_value='1',
                          description='The DDS domain of unitree_sdk2 (the real robots use 0)'),
    OpaqueFunction(function=launch_setup),
  ])
