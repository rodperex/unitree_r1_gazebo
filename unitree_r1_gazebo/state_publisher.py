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

"""r1_state_publisher: the state of the R1, from the DDS of unitree_sdk2 to ROS, the same for
the simulated robot (r1_sdk_sim) and the real one:

    rt/lowstate (unitree_hg LowState_)              -> /joint_states (sensor_msgs/JointState)
    rt/odommodestate (unitree_go SportModeState_)   -> TF odom -> base_link, /odom (nav_msgs/Odometry)

With robot_state_publisher and the description URDF (model.description_urdf), the whole TF tree
of the robot. base_link is on the floor under the pelvis.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from unitree_r1_gazebo.joints import MOTORS


class R1StatePublisher(Node):

    def __init__(self):
        super().__init__('r1_state_publisher')
        p = {param.name: param.value for param in self.declare_parameters('', [
            ('interface', 'lo'),
            ('domain', 1),
            ('odometry_topic', 'rt/odommodestate'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('publish_tf', True),
            ('rate', 50.0),          # Hz: at most, of /joint_states and the odometry
        ])}
        self.p = p
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.period = 1.0 / p['rate']
        self.last = {'joints': 0.0, 'odometry': 0.0}

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        ChannelFactoryInitialize(p['domain'], p['interface'])
        self.lowstate_sub = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_sub.Init(self.on_lowstate, 10)
        self.odometry_sub = ChannelSubscriber(p['odometry_topic'], SportModeState_)
        self.odometry_sub.Init(self.on_odometry, 10)
        self.get_logger().info(f'The state of the R1 from DDS domain {p["domain"]} on '
                               f'{p["interface"]}')

    def due(self, what):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last[what] < self.period and now >= self.last[what]:
            return False
        self.last[what] = now
        return True

    def on_lowstate(self, msg):
        if not self.due('joints'):
            return
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        for index, joint in MOTORS.items():
            state.name.append(joint)
            state.position.append(float(msg.motor_state[index].q))
            state.velocity.append(float(msg.motor_state[index].dq))
        self.joint_pub.publish(state)

    def on_odometry(self, msg):
        if not self.due('odometry'):
            return
        stamp = self.get_clock().now().to_msg()
        yaw = float(msg.imu_state.rpy[2])
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.p['odom_frame']
        odometry.child_frame_id = self.p['base_frame']
        odometry.pose.pose.position.x = float(msg.position[0])
        odometry.pose.pose.position.y = float(msg.position[1])
        odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w = qz, qw
        # The twist in the frame of the body
        vx, vy = float(msg.velocity[0]), float(msg.velocity[1])
        odometry.twist.twist.linear.x = vx * math.cos(yaw) + vy * math.sin(yaw)
        odometry.twist.twist.linear.y = -vx * math.sin(yaw) + vy * math.cos(yaw)
        odometry.twist.twist.angular.z = float(msg.yaw_speed)
        self.odom_pub.publish(odometry)
        if self.p['publish_tf']:
            transform = TransformStamped()
            transform.header = odometry.header
            transform.child_frame_id = self.p['base_frame']
            transform.transform.translation.x = odometry.pose.pose.position.x
            transform.transform.translation.y = odometry.pose.pose.position.y
            transform.transform.rotation = odometry.pose.pose.orientation
            self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = R1StatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
