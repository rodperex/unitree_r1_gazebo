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

"""r1_sdk_sim: the firmware of the R1 for the simulated robot. It speaks the DDS interface of
unitree_sdk2, as the real robot does, so that a program written with the SDK moves the robot in
Gazebo without a change (only the network interface and the DDS domain differ):

    the locomotion service "sport"  LocoClient (unitree_sdk2py.r1.loco): SetVelocity / Move,
                                    StopMove, SetFsmId (Damp, Stance, Start...), GetFsmId
    rt/arm_sdk   (unitree_hg LowCmd_)  the arms, the waist yaw and the head, by position,
                                       blended with their rest by the weight (mode_pr, 0-100)
    rt/lowstate  (unitree_hg LowState_)  the joints and the IMU, published
    rt/odommodestate (unitree_go SportModeState_)  the position and heading of the body

It moves the simulated robot through Gazebo transport (the joints of model.simulation_urdf).
The locomotion is not the robot's: the body slides and turns, reaching the commanded velocity
with limited accelerations, which stands for the time the real controller takes, while the
legs step with strides that match its velocity (an animation, gait.py) and, unless the arm SDK
holds them, the arms swing.

    ros2 run unitree_r1_gazebo r1_sdk_sim --interface lo --domain 1
"""

import argparse
import json
import math
import signal
import threading
import time

from unitree_r1_gazebo.crc import crc_function
from unitree_r1_gazebo.gait import Gait
from unitree_r1_gazebo.joints import ARM_SDK, BASE_X, BASE_Y, BODY_YAW, INDEX, LEGS, MOTORS

# Locomotion states (SetFsmId) in which the robot walks: Start (locomotion mode)
WALKING_FSM = 811
# Codes of the replies (0: done)
OK, NOT_WALKING = 0, 7301


class Locomotion:
    """The velocity of the body, towards the commanded one with limited accelerations. No
    ROS, no DDS: the state of the emulated controller."""

    def __init__(self, max_speed=(0.8, 0.4, 1.5), max_accel=(0.8, 0.8, 2.0), fsm=WALKING_FSM):
        self.max_speed, self.max_accel = max_speed, max_accel
        self.fsm = fsm
        self.command = (0.0, 0.0, 0.0)
        self.until = 0.0                  # the command holds until then (s)
        self.velocity = [0.0, 0.0, 0.0]   # vx, vy (body frame, m/s), yaw rate (rad/s)

    def set_velocity(self, vx, vy, wz, duration, now):
        if self.fsm != WALKING_FSM:
            return NOT_WALKING
        self.command = tuple(max(-m, min(m, v)) for v, m in zip((vx, vy, wz), self.max_speed))
        self.until = now + max(0.0, duration)
        return OK

    def set_fsm(self, fsm):
        self.fsm = int(fsm)
        if self.fsm != WALKING_FSM:     # damping, standing still...: no velocity
            self.command, self.until = (0.0, 0.0, 0.0), 0.0
        return OK

    def step(self, now, dt):
        """The velocity after dt seconds."""
        target = self.command if now < self.until else (0.0, 0.0, 0.0)
        for i in range(3):
            change = max(-self.max_accel[i] * dt, min(self.max_accel[i] * dt,
                                                       target[i] - self.velocity[i]))
            self.velocity[i] += change
        return tuple(self.velocity)


def world_velocity(velocity, yaw):
    """The velocity of the body (vx, vy in its frame) in the axes of the world."""
    vx, vy, wz = velocity
    return vx * math.cos(yaw) - vy * math.sin(yaw), vx * math.sin(yaw) + vy * math.cos(yaw), wz


def arm_targets(command_q, weight, rest):
    """The position of each arm SDK joint: the command blended with the rest by the weight."""
    w = max(0.0, min(1.0, weight))
    return {joint: w * command_q.get(joint, rest.get(joint, 0.0)) + (1.0 - w) * rest.get(joint, 0.0)
            for joint in ARM_SDK}


class R1SdkSim:

    def __init__(self, args):
        # Imported here: the tests of the pure parts above need neither DDS nor Gazebo
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node as GzNode
        from unitree_sdk2py.core.channel import (ChannelFactoryInitialize, ChannelPublisher,
                                                 ChannelSubscriber)
        from unitree_sdk2py.idl.default import (unitree_go_msg_dds__SportModeState_,
                                                unitree_hg_msg_dds__LowState_)
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_r1_gazebo.r1_loco import (
            LOCO_API_VERSION, LOCO_SERVICE_NAME, ROBOT_API_ID_LOCO_GET_FSM_ID,
            ROBOT_API_ID_LOCO_SET_ARM_TASK, ROBOT_API_ID_LOCO_SET_FSM_ID,
            ROBOT_API_ID_LOCO_SET_VELOCITY)
        from unitree_sdk2py.rpc.server import Server

        self.args = args
        self.Double = Double
        self.new_lowstate = unitree_hg_msg_dds__LowState_
        self.new_odometry = unitree_go_msg_dds__SportModeState_
        # The CRC of rt/lowstate only with the compiled library of the SDK (else 0): at 100 Hz
        self.crc = crc_function(compiled_only=True)
        self.lock = threading.Lock()
        self.loco = Locomotion((args.max_vx, args.max_vy, args.max_wz),
                               (args.accel_linear, args.accel_linear, args.accel_angular),
                               args.initial_fsm)
        self.positions, self.velocities = {}, {}
        self.arm_q, self.arm_weight = {}, 0.0
        self.rest = {joint: 0.0 for joint in ARM_SDK}
        self.gait = Gait()
        self.tick = 0

        # Gazebo
        self.gz = GzNode()
        prefix = f'/{args.name}'
        self.gz.subscribe(Model, f'{prefix}/joint_state', self.on_joint_state)
        self.cmd_pos = {j: self.gz.advertise(f'{prefix}/{j}/cmd_pos', Double)
                        for j in (*ARM_SDK, *LEGS)}
        self.cmd_vel = {j: self.gz.advertise(f'{prefix}/{j}/cmd_vel', Double)
                        for j in (BASE_X, BASE_Y, BODY_YAW)}

        # The DDS of unitree_sdk2
        ChannelFactoryInitialize(args.domain, args.interface)
        self.lowstate_pub = ChannelPublisher('rt/lowstate', LowState_)
        self.lowstate_pub.Init()
        self.odometry_pub = ChannelPublisher(args.odometry_topic, SportModeState_)
        self.odometry_pub.Init()
        self.arm_sub = ChannelSubscriber('rt/arm_sdk', LowCmd_)
        self.arm_sub.Init(self.on_arm_sdk, 10)

        loco = self.loco
        sim = self

        class Sport(Server):
            def __init__(self):
                super().__init__(LOCO_SERVICE_NAME)

            def Init(self):
                self._SetApiVersion(LOCO_API_VERSION)
                self._RegistHandler(ROBOT_API_ID_LOCO_GET_FSM_ID,
                                    lambda p: (OK, json.dumps({'data': loco.fsm})), False)
                self._RegistHandler(ROBOT_API_ID_LOCO_SET_FSM_ID, sim.on_set_fsm, False)
                self._RegistHandler(ROBOT_API_ID_LOCO_SET_VELOCITY, sim.on_set_velocity, False)
                self._RegistHandler(ROBOT_API_ID_LOCO_SET_ARM_TASK, lambda p: (OK, ''), False)

        self.server = Sport()
        self.server.Init()
        self.server.Start(False)
        print(f'r1_sdk_sim: DDS domain {args.domain} on {args.interface}; locomotion state '
              f'{loco.fsm}; Gazebo model /{args.name}', flush=True)

    # --- Gazebo
    def on_joint_state(self, msg):
        with self.lock:
            for joint in msg.joint:
                self.positions[joint.name] = joint.axis1.position
                self.velocities[joint.name] = joint.axis1.velocity

    # --- unitree_sdk2
    def on_set_velocity(self, parameter):
        p = json.loads(parameter)
        vx, vy, wz = (list(p.get('velocity', [0.0, 0.0, 0.0])) + [0.0, 0.0, 0.0])[:3]
        with self.lock:
            return self.loco.set_velocity(float(vx), float(vy), float(wz),
                                          float(p.get('duration', 1.0)), time.monotonic()), ''

    def on_set_fsm(self, parameter):
        with self.lock:
            return self.loco.set_fsm(json.loads(parameter).get('data', 0)), ''

    def on_arm_sdk(self, msg):
        with self.lock:
            self.arm_weight = msg.mode_pr / 100.0
            self.arm_q = {joint: msg.motor_cmd[INDEX[joint]].q for joint in ARM_SDK}

    # --- the loop
    def step(self, dt):
        now = time.monotonic()
        with self.lock:
            velocity = self.loco.step(now, dt)
            yaw = self.positions.get(BODY_YAW, 0.0)
            self.gait.step(velocity, dt)
            # The rest pose of the arms swings with the gait: seen where the arm SDK lets it
            rest = dict(self.rest, **{j: self.rest[j] + v for j, v in self.gait.arm_swing().items()})
            targets = arm_targets(self.arm_q, self.arm_weight, rest)
            targets.update(self.gait.legs())
            positions, velocities = dict(self.positions), dict(self.velocities)
        vx, vy, wz = world_velocity(velocity, yaw)
        for joint, value in ((BASE_X, vx), (BASE_Y, vy), (BODY_YAW, wz)):
            self.cmd_vel[joint].publish(self.Double(data=value))
        for joint, value in targets.items():
            self.cmd_pos[joint].publish(self.Double(data=value))
        self.publish_state(positions, velocities, velocity, yaw)

    def publish_state(self, positions, velocities, velocity, yaw):
        self.tick += 1
        state = self.new_lowstate()
        state.tick = self.tick
        for index, joint in MOTORS.items():
            state.motor_state[index].q = float(positions.get(joint, 0.0))
            state.motor_state[index].dq = float(velocities.get(joint, 0.0))
        state.imu_state.rpy = [0.0, 0.0, float(yaw)]
        state.imu_state.quaternion = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        state.imu_state.gyroscope = [0.0, 0.0, float(velocity[2])]
        state.crc = self.crc(state) if self.crc else 0
        self.lowstate_pub.Write(state)
        odometry = self.new_odometry()
        odometry.position = [float(positions.get(BASE_X, 0.0)), float(positions.get(BASE_Y, 0.0)),
                             0.0]
        wx, wy, _ = world_velocity(velocity, yaw)
        odometry.velocity = [float(wx), float(wy), 0.0]
        odometry.yaw_speed = float(velocity[2])
        odometry.imu_state.rpy = [0.0, 0.0, float(yaw)]
        odometry.imu_state.quaternion = state.imu_state.quaternion
        self.odometry_pub.Write(odometry)

    def stop(self):
        """The body still. The joint controllers of Gazebo hold the last velocity they were given:
        without this, the robot would keep sliding after this program ends."""
        for _ in range(5):
            for publisher in self.cmd_vel.values():
                publisher.publish(self.Double(data=0.0))
            time.sleep(0.02)

    def spin(self):
        period = 1.0 / self.args.rate
        last = time.monotonic()
        while True:
            now = time.monotonic()
            self.step(now - last)
            last = now
            time.sleep(max(0.0, period - (time.monotonic() - now)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--name', default='r1', help='The model in Gazebo (its topics)')
    parser.add_argument('--interface', default='lo', help='Network interface of the SDK DDS')
    parser.add_argument('--domain', type=int, default=1, help='DDS domain of the SDK')
    parser.add_argument('--rate', type=float, default=100.0, help='Hz of the control and state')
    parser.add_argument('--initial-fsm', type=int, default=WALKING_FSM,
                        help='Locomotion state at the start (811: walking, as after Start)')
    parser.add_argument('--max-vx', type=float, default=0.8)
    parser.add_argument('--max-vy', type=float, default=0.4)
    parser.add_argument('--max-wz', type=float, default=1.5)
    parser.add_argument('--accel-linear', type=float, default=0.8, help='m/s^2')
    parser.add_argument('--accel-angular', type=float, default=2.0, help='rad/s^2')
    parser.add_argument('--odometry-topic', default='rt/odommodestate')
    args, _ = parser.parse_known_args(argv)   # the ROS arguments of a launch file are ignored
    # SIGTERM (a launch that escalates) ends it as Ctrl-C does: stopping the body first
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    sim = None
    try:
        sim = R1SdkSim(args)
        sim.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if sim is not None:
            sim.stop()
