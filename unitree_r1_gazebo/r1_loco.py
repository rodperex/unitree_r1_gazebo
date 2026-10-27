# The locomotion client of the Unitree R1, from unitree_sdk2_python (unitree_sdk2py/r1/loco,
# commit 814556d15970dd2ecf1c9984e845ca02ab07e206):
#
#   Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
#   All rights reserved. BSD 3-Clause License: see LICENSE.unitree in this package.
#
# The installed unitree_sdk2py lacks its r1 package (unitree_sdk2py/r1 has no __init__.py, so its
# setup.py does not install it). Its client is used when it is there; otherwise this copy, which
# is the same code with absolute imports. Only the module name differs; the service, the API
# and the client are Unitree's.

import json

try:
    from unitree_sdk2py.r1.loco.r1_loco_api import (  # noqa: F401
        LOCO_API_VERSION, LOCO_SERVICE_NAME, ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_ARM_TASK, ROBOT_API_ID_LOCO_SET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_VELOCITY)
    from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient  # noqa: F401
except ImportError:
    from unitree_sdk2py.rpc.client import Client

    # service name
    LOCO_SERVICE_NAME = "sport"
    # service api version
    LOCO_API_VERSION = "1.0.0.0"
    # api id
    ROBOT_API_ID_LOCO_GET_FSM_ID = 7001
    ROBOT_API_ID_LOCO_SET_FSM_ID = 7101
    ROBOT_API_ID_LOCO_SET_VELOCITY = 7105
    ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106

    class LocoClient(Client):
        def __init__(self):
            super().__init__(LOCO_SERVICE_NAME, False)
            self.first_shake_hand_stage_ = -1

        def Init(self):
            # set api version
            self._SetApiVerson(LOCO_API_VERSION)
            # regist api
            self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
            self._RegistApi(ROBOT_API_ID_LOCO_SET_FSM_ID, 0)
            self._RegistApi(ROBOT_API_ID_LOCO_SET_VELOCITY, 0)
            self._RegistApi(ROBOT_API_ID_LOCO_SET_ARM_TASK, 0)

        # 7101
        def SetFsmId(self, fsm_id: int):
            p = {}
            p["data"] = fsm_id
            parameter = json.dumps(p)
            code, data = self._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, parameter)
            return code

        # 7105
        def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
            p = {}
            velocity = [vx, vy, omega]
            p["velocity"] = velocity
            p["duration"] = duration
            parameter = json.dumps(p)
            code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)
            return code

        # 7106
        def SetTaskId(self, task_id: float):
            p = {}
            p["data"] = task_id
            parameter = json.dumps(p)
            code, data = self._Call(ROBOT_API_ID_LOCO_SET_ARM_TASK, parameter)
            return code

        def Damp(self):
            self.SetFsmId(1)

        def Stance(self):
            self.SetFsmId(4)

        def Start(self):
            self.SetFsmId(811)

        def Lie2StandUp(self):
            self.SetFsmId(701)

        def StandUp2Lie(self):
            self.SetFsmId(702)

        def ZeroTorque(self):
            self.SetFsmId(0)

        def StopMove(self):
            self.SetVelocity(0., 0., 0.)

        def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
            duration = 864000.0 if continous_move else 1
            self.SetVelocity(vx, vy, vyaw, duration)

        def WaveHand(self, turn_flag: bool = False):
            self.SetTaskId(1 if turn_flag else 0)

        def ShakeHand(self, stage: int = -1):
            if stage == 0:
                self.first_shake_hand_stage_ = False
                self.SetTaskId(2)
            elif stage == 1:
                self.first_shake_hand_stage_ = True
                self.SetTaskId(3)
            else:
                self.first_shake_hand_stage_ = not self.first_shake_hand_stage_
                return self.SetTaskId(3 if self.first_shake_hand_stage_ else 2)
