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

import math
import os
import xml.etree.ElementTree as ET

import pytest

from unitree_r1_gazebo import joints, model
from unitree_r1_gazebo.sdk_sim import (NOT_WALKING, OK, WALKING_FSM, Locomotion, arm_targets,
                                       world_velocity)


def test_the_velocity_is_reached_with_limited_acceleration():
    loco = Locomotion(max_speed=(1.0, 0.5, 1.5), max_accel=(0.5, 0.5, 2.0))
    assert loco.set_velocity(0.4, 0.0, 0.0, 10.0, now=0.0) == OK
    vx = [loco.step(now=0.1 * k, dt=0.1)[0] for k in range(1, 11)]
    assert vx[0] == pytest.approx(0.05) and vx[-1] == pytest.approx(0.4)


def test_a_command_holds_for_its_duration_then_the_body_stops():
    loco = Locomotion(max_accel=(10.0, 10.0, 10.0))
    loco.set_velocity(0.3, 0.0, 0.5, 1.0, now=0.0)
    assert loco.step(now=0.5, dt=0.5) == pytest.approx((0.3, 0.0, 0.5))
    assert loco.step(now=1.5, dt=1.0) == pytest.approx((0.0, 0.0, 0.0))


def test_the_speed_is_capped():
    loco = Locomotion(max_speed=(0.8, 0.4, 1.5), max_accel=(100.0, 100.0, 100.0))
    loco.set_velocity(5.0, -5.0, 5.0, 1.0, now=0.0)
    assert loco.step(now=0.1, dt=0.1) == pytest.approx((0.8, -0.4, 1.5))


def test_it_walks_only_in_the_walking_state():
    loco = Locomotion()
    loco.set_fsm(1)   # damp
    assert loco.set_velocity(0.3, 0.0, 0.0, 1.0, now=0.0) == NOT_WALKING
    loco.set_fsm(WALKING_FSM)
    assert loco.set_velocity(0.3, 0.0, 0.0, 1.0, now=0.0) == OK


def test_world_velocity_rotates_with_the_heading():
    vx, vy, wz = world_velocity((1.0, 0.0, 0.2), math.pi / 2)
    assert (vx, vy, wz) == pytest.approx((0.0, 1.0, 0.2))


def test_the_arm_sdk_blends_with_the_rest_by_its_weight():
    rest = {j: 0.0 for j in joints.ARM_SDK}
    command = {'head_yaw_joint': 0.6}
    assert arm_targets(command, 1.0, rest)['head_yaw_joint'] == pytest.approx(0.6)
    assert arm_targets(command, 0.5, rest)['head_yaw_joint'] == pytest.approx(0.3)
    assert arm_targets(command, 0.0, rest)['head_yaw_joint'] == 0.0


def test_the_joints_of_the_sdk():
    assert joints.INDEX['waist_yaw_joint'] == 13
    assert joints.INDEX['head_pitch_joint'] == 29 and joints.INDEX['head_yaw_joint'] == 30
    assert set(joints.ARM_SDK) <= set(joints.MOTORS.values())
    assert max(joints.MOTORS) < joints.NUM_MOTORS


@pytest.mark.skipif(not os.path.isfile(os.path.join(model.CACHE, 'R1.urdf')),
                    reason='the model of Unitree is not downloaded (fetch_r1_model)')
def test_the_simulated_robot_moves_only_what_the_firmware_moves():
    robot = ET.fromstring(model.simulation_urdf())
    movable = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
    assert movable == set(model.POSITION_JOINTS) | set(model.VELOCITY_JOINTS)
    assert not robot.findall('.//collision')
    description = ET.fromstring(model.description_urdf())
    assert {'base_link', 'pelvis_link', 'camera_link'} <= {l.get('name') for l in description.findall('link')}
    assert 'world' not in {l.get('name') for l in description.findall('link')}


def test_the_legs_are_straight_standing_and_step_walking():
    from unitree_r1_gazebo.gait import Gait
    gait = Gait()
    gait.step((0.0, 0.0, 0.0), 0.1)
    assert all(abs(v) < 1e-9 for v in gait.legs().values())
    for _ in range(45):   # 2.25 s: a quarter of a cycle into the third, a leg in the air
        gait.step((0.4, 0.0, 0.0), 0.05)
    knees = [abs(gait.legs()['left_knee_joint']), abs(gait.legs()['right_knee_joint'])]
    assert max(knees) > 0.1   # one leg in the air
    seen = set()
    for _ in range(40):
        gait.step((0.4, 0.0, 0.0), 0.05)
        seen.add(gait.legs()['left_hip_pitch_joint'] < 0)
    assert seen == {True, False}   # the leg goes in front and behind


def test_the_stance_leg_sweeps_back_as_the_body_goes_forward():
    import math
    from unitree_r1_gazebo.gait import Gait
    gait = Gait()
    gait.amount, gait.velocity = 1.0, (0.4, 0.0, 0.0)
    gait.phase = math.pi            # the left foot lands, in front
    front = gait.legs()['left_hip_pitch_joint']
    gait.phase = 1.9 * math.pi      # late in the stance, behind
    assert front < 0.0 < gait.legs()['left_hip_pitch_joint']
    assert gait.legs()['left_knee_joint'] == 0.0   # straight on the floor
