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

"""The joints of the Unitree R1: their index in the messages of unitree_sdk2 (LowCmd_ and
LowState_ of unitree_hg; include/unitree/dds_wrapper/robots/r1/defines.h) and their name in the
URDF of Unitree (unitree_ros, robots/r1_description). No ROS here."""

# Motor index (unitree_sdk2) -> URDF joint
MOTORS = {
    0: 'left_hip_pitch_joint', 1: 'left_hip_roll_joint', 2: 'left_hip_yaw_joint',
    3: 'left_knee_joint', 4: 'left_ankle_pitch_joint', 5: 'left_ankle_roll_joint',
    6: 'right_hip_pitch_joint', 7: 'right_hip_roll_joint', 8: 'right_hip_yaw_joint',
    9: 'right_knee_joint', 10: 'right_ankle_pitch_joint', 11: 'right_ankle_roll_joint',
    12: 'waist_roll_joint', 13: 'waist_yaw_joint',
    15: 'left_shoulder_pitch_joint', 16: 'left_shoulder_roll_joint', 17: 'left_shoulder_yaw_joint',
    18: 'left_elbow_joint', 19: 'left_wrist_roll_joint',
    22: 'right_shoulder_pitch_joint', 23: 'right_shoulder_roll_joint',
    24: 'right_shoulder_yaw_joint', 25: 'right_elbow_joint', 26: 'right_wrist_roll_joint',
    29: 'head_pitch_joint', 30: 'head_yaw_joint',
}
INDEX = {name: index for index, name in MOTORS.items()}
NUM_MOTORS = 35   # the size of motor_cmd / motor_state in unitree_hg

# The joints that the arm SDK (rt/arm_sdk) commands while the locomotion controller walks:
# both arms, the waist yaw and the head (ArmSdk::JOINTS of unitree_sdk2)
ARM_SDK = [
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint',
    'waist_yaw_joint', 'head_pitch_joint', 'head_yaw_joint',
]

# The joints of the legs: moved by the gait of the simulation (gait.py) while the body walks
LEGS = [MOTORS[i] for i in range(12)]

# The joints of the simulated body over the floor (not in the robot): the locomotion moves them
BASE_X, BASE_Y, BODY_YAW = 'base_x_joint', 'base_y_joint', 'body_yaw_joint'
