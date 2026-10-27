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

"""An animation of the gait of the R1: the angles of its legs (and the swing of its arms) while
its body moves, so that the simulated robot steps instead of sliding. It is not a controller:
the body is moved by the locomotion of the simulation (sdk_sim), and the legs only follow it,
with strides that match its velocity, so that the feet barely slip. No ROS here.

Each leg goes through a cycle of period `period`: half of it in the air (swing: the foot from
behind to in front, the knee bent to lift it) and half on the floor (stance: the straight leg
from in front to behind, as the body passes over it). The legs are half a cycle apart. The
stride follows the velocity of the body: forward (hip pitch), sideways (hip roll) and turning
(hip yaw); the ankle keeps the foot level. Standing still, the legs are straight.

Joint conventions (the URDF of Unitree): hip pitch > 0 takes the foot back, knee > 0 bends it,
hip roll > 0 takes the foot to the left, hip yaw > 0 turns it to the left.
"""

import math

LEG_LENGTH = 0.68      # m: from the hip to the sole, standing (the pelvis 0.72 m over the floor)
PERIOD = 1.0           # s: a full cycle (two steps)
LIFT = 0.6             # rad: the bend of the knee in the middle of a swing
MAX_STRIDE_ANGLE = 0.45
MAX_TURN_ANGLE = 0.4
MAX_ANKLE_PITCH = (-0.87, 0.58)
MAX_ANKLE_ROLL = 0.26
ARM_SWING = 0.8        # the shoulder swings this much of the hip of the other leg


def clamp(value, low, high):
    return max(low, min(high, value))


class Gait:
    """The phase of the cycle and how much the legs step (0: standing, 1: walking)."""

    def __init__(self, period=PERIOD, ramp=3.0):
        self.period = period
        self.ramp = ramp          # 1/s: how fast the steps start and stop
        self.phase = 0.0          # rad, of the left leg
        self.amount = 0.0
        self.velocity = (0.0, 0.0, 0.0)

    def step(self, velocity, dt):
        """Advance by dt seconds with the body at this velocity (vx, vy in its frame, m/s; yaw
        rate, rad/s)."""
        vx, vy, wz = velocity
        moving = abs(vx) > 0.02 or abs(vy) > 0.02 or abs(wz) > 0.05
        target = 1.0 if moving else 0.0
        self.amount += clamp(target - self.amount, -self.ramp * dt, self.ramp * dt)
        if moving:
            self.velocity = velocity
        if self.amount > 0.0:
            self.phase = (self.phase + 2.0 * math.pi * dt / self.period) % (2.0 * math.pi)
        elif not moving:
            self.phase = 0.0

    def legs(self):
        """The angles of the joints of both legs (URDF names)."""
        vx, vy, wz = self.velocity
        a = self.amount
        # Over a cycle the body moves v * period: each leg sweeps that, from front to back
        pitch = clamp(math.atan2(vx * self.period / 2.0, LEG_LENGTH), -MAX_STRIDE_ANGLE, MAX_STRIDE_ANGLE)
        roll = clamp(math.atan2(vy * self.period / 2.0, LEG_LENGTH), -MAX_STRIDE_ANGLE, MAX_STRIDE_ANGLE)
        turn = clamp(wz * self.period / 2.0, -MAX_TURN_ANGLE, MAX_TURN_ANGLE)
        angles = {}
        for side, phase in (('left', self.phase), ('right', self.phase + math.pi)):
            swing = math.sin(phase) if math.sin(phase) > 0.0 else 0.0    # in the air
            # cos(phase): +1 (behind) at the start of the swing, -1 (in front) at its end
            hip_pitch = a * (pitch * math.cos(phase)) - 0.5 * a * LIFT * swing
            knee = a * LIFT * swing
            hip_roll = -a * roll * math.cos(phase)
            hip_yaw = -a * turn * math.cos(phase)
            angles[f'{side}_hip_pitch_joint'] = hip_pitch
            angles[f'{side}_knee_joint'] = knee
            angles[f'{side}_ankle_pitch_joint'] = clamp(-(hip_pitch + knee), *MAX_ANKLE_PITCH)
            angles[f'{side}_hip_roll_joint'] = hip_roll
            angles[f'{side}_ankle_roll_joint'] = clamp(-hip_roll, -MAX_ANKLE_ROLL, MAX_ANKLE_ROLL)
            angles[f'{side}_hip_yaw_joint'] = hip_yaw
        return angles

    def arm_swing(self):
        """The shoulder pitch of each arm: forward with the other leg, as people walk."""
        legs = self.legs()
        return {'left_shoulder_pitch_joint': ARM_SWING * legs['right_hip_pitch_joint'],
                'right_shoulder_pitch_joint': ARM_SWING * legs['left_hip_pitch_joint']}
