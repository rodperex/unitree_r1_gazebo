# unitree_r1_gazebo

The [Unitree R1](https://www.unitree.com/R1) humanoid in Gazebo (Harmonic, ROS 2 Jazzy), driven
**as the real robot**: through the official SDK, [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2_python).
A program that moves the R1 with the SDK (`LocoClient.Move` to walk and turn, `rt/arm_sdk` for the
head, the waist and the arms, `rt/lowstate` to read the joints) moves the simulated R1 without a
change: only the network interface and the DDS domain differ.

```text
   your program (unitree_sdk2)                 Gazebo
   LocoClient.Move(vx, vy, vyaw)  ──┐      ┌──> the R1: its body moves and turns,
   rt/arm_sdk (head, waist, arms) ──┼──>  r1_sdk_sim  (the firmware)    its head, waist and arms move,
   rt/lowstate, odometry  <─────────┘      └──< its joints, its RGB-D head camera
                                                     │
   ROS 2:  /joint_states, /odom, TF  <── r1_state_publisher  (also on the real robot)
           /r1/camera/{image,depth_image,camera_info}  <── ros_gz_bridge
```

It also has worlds to put the robot in: an empty one, a warehouse, a house and a hospital room;
or any world of yours.

## Contents

- [What is simulated, and what is not](#what-is-simulated-and-what-is-not)
- [Installation](#installation)
- [Quick start](#quick-start)
- [The launch file](#the-launch-file)
- [The worlds](#the-worlds)
- [Driving the robot (unitree_sdk2)](#driving-the-robot-unitree_sdk2)
- [What it publishes in ROS](#what-it-publishes-in-ros)
- [On the real robot](#on-the-real-robot)
- [Troubleshooting](#troubleshooting)

## What is simulated, and what is not

| Part | In the simulation |
|---|---|
| Walking and turning (`LocoClient`) | The body **slides and turns** at the commanded velocity, reached with limited accelerations (0.8 m/s², 2 rad/s²) and capped (0.8 m/s forward, 0.4 m/s sideways, 1.5 rad/s): the delay of the real controller, not its steps. The legs do not move. Only in the walking state (`Start`, FSM 811, the initial state); `Damp`, `Stance`... stop it |
| Arms, waist yaw, head (`rt/arm_sdk`) | By position, at most at 2 rad/s (waist 1, head pitch 1.5), blended with the rest pose by the weight (`mode_pr`, 0-100), as on the robot |
| Joint and IMU state (`rt/lowstate`) | The joints above (the legs and the waist roll at 0), the heading of the body, its yaw rate |
| Odometry (`rt/odommodestate`) | The position and heading of the body from where it started (exact: no drift) |
| Head camera | RGB-D, 640 × 480, 69° horizontal field of view, 15 Hz (the real one: to be checked) |
| Physics | None for the robot: it has no gravity and no collisions, and it goes through walls. The objects of the world are as their models say |
| Hands, audio, low-level control (`rt/lowcmd`), arm actions (`SetTaskId`) | Not simulated (`SetTaskId` is accepted and ignored) |

This is enough to test what a program decides (where the robot looks, how it turns, how it
approaches someone) with the interface of the real robot. To test walking itself (balance,
steps, falls), use the official MuJoCo simulator,
[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco), with a locomotion policy.

## Installation

In the `src` folder of a ROS 2 Jazzy workspace with Gazebo Harmonic (`ros_gz`):

```sh
git clone https://github.com/rodperex/unitree_r1_gazebo.git
cd .. && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select unitree_r1_gazebo
```

**The official SDK.** `unitree_sdk2_python` and the Python binding of CycloneDDS are not ROS
packages. The binding must be built against the CycloneDDS of your ROS (one CycloneDDS library
per process, also with `rmw_cyclonedds_cpp`); ROS 2 Jazzy ships CycloneDDS 0.10.5:

```sh
# The binding of CycloneDDS, against the library of ROS
CYCLONEDDS_HOME=/opt/ros/jazzy pip install --no-build-isolation cyclonedds==0.10.5
# The SDK (it asks for cyclonedds 0.10.2: --no-deps keeps the 0.10.5 above)
pip install --no-deps git+https://github.com/unitreerobotics/unitree_sdk2_python.git@814556d15970dd2ecf1c9984e845ca02ab07e206
```

With [pixi](https://pixi.sh), in the `pixi.toml` of your project:

```toml
[pypi-dependencies]
unitree_sdk2py = { git = "https://github.com/unitreerobotics/unitree_sdk2_python.git", rev = "814556d15970dd2ecf1c9984e845ca02ab07e206" }
cyclonedds = "==0.10.5"

[activation.env]
CYCLONEDDS_HOME = "$CONDA_PREFIX"

[pypi-options]
no-build-isolation = ["cyclonedds"]
dependency-overrides = { cyclonedds = "==0.10.5" }
```

The installed SDK lacks its R1 package (`unitree_sdk2py/r1` has no `__init__.py`, so its
`setup.py` leaves it out). This package brings the R1 locomotion client, Unitree's code (BSD-3,
`LICENSE.unitree`): `from unitree_r1_gazebo.r1_loco import LocoClient`; it is the SDK's own when
the SDK has it.

**The model** of the R1 (Unitree's URDF and meshes, `unitree_ros`, BSD-3) is downloaded the first
time, at a fixed commit, into `~/.cache/unitree_r1_gazebo`; `ros2 run unitree_r1_gazebo
fetch_r1_model` downloads it beforehand. The models of the worlds come from
[Gazebo Fuel](https://app.gazebosim.org) and are downloaded the first time too.

## Quick start

```sh
ros2 launch unitree_r1_gazebo r1_gazebo.launch.py world:=house
```

Then, from another terminal, walk with the SDK (domain 1 on the loopback interface, the defaults
of the simulation):

```python
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_r1_gazebo.r1_loco import LocoClient

ChannelFactoryInitialize(1, 'lo')       # on the real robot: 0 and its network interface
loco = LocoClient()
loco.SetTimeout(3.0)
loco.Init()
for _ in range(20):                      # each Move lasts 1 s: repeat it to keep walking
    loco.Move(0.3, 0.0, 0.0)             # forward at 0.3 m/s
    time.sleep(0.1)
loco.Move(0.0, 0.0, 0.8)                 # turn left
```

`ros2 topic echo /odom`, RViz with the TF and `/r1/camera/image`, or the Gazebo window, show it.

## The launch file

`ros2 launch unitree_r1_gazebo r1_gazebo.launch.py`

| Argument | Default | Meaning |
|---|---|---|
| `world` | `empty` | `empty`, `depot`, `house`, `hospital_room`, or the path of a world (SDF, or an SDF xacro with an argument `headless`) |
| `headless` | `false` | `true`: no Gazebo window (the simulation runs the same, also the camera) |
| `name` | `r1` | The model in Gazebo and the prefix of its topics |
| `x`, `y`, `yaw` | `0.0` | Where the robot starts (m, rad), in the world; the odometry starts there |
| `sdk_interface` | `lo` | The network interface of the DDS of unitree_sdk2 |
| `sdk_domain` | `1` | The DDS domain of unitree_sdk2 (the real robots use 0: another one keeps a simulation apart from a robot on the same network) |

It starts Gazebo, the R1, `r1_sdk_sim`, `r1_state_publisher`, `robot_state_publisher` and a
bridge of `/clock` and the camera. Everything uses the simulation clock.

`r1_sdk_sim` also takes, as arguments of `ros2 run unitree_r1_gazebo r1_sdk_sim`: `--max-vx`,
`--max-vy`, `--max-wz`, `--accel-linear`, `--accel-angular`, `--initial-fsm`, `--rate` and
`--odometry-topic`.

## The worlds

| World | What | Where the robot starts |
|---|---|---|
| `empty` | The floor and the sun | |
| `depot` | The Depot warehouse of Gazebo Fuel (OpenRobotics/Depot) | In its middle |
| `house` | 9 × 7 m: a living room (sofa, coffee table, TV stand, bookshelf, lamp), a kitchen-dining room (counter, fridge, table and chairs) and a bedroom (bed, bedside table) | In the living room, facing the kitchen |
| `hospital_room` | 6 × 5 m: a hospital bed, a bedside table, an IV stand, a visitor chair and a cart | By the door, facing the bed |

Your own world: `world:=/path/to/world.sdf`. It needs the systems that the robot uses: Physics,
UserCommands, SceneBroadcaster and Sensors (with `ogre2`); see `worlds/empty.sdf`.

## Driving the robot (unitree_sdk2)

The same calls as on the real robot. The defaults of the simulation: DDS domain 1, interface `lo`.

**Walking and turning:** `LocoClient` (`from unitree_r1_gazebo.r1_loco import LocoClient`, or
`unitree_sdk2py.r1.loco.r1_loco_client` when your SDK has it), the service `sport`:

| Call | In the simulation |
|---|---|
| `Move(vx, vy, vyaw)` | Velocity in the frame of the body (m/s, rad/s) for 1 s (`continous_move=True`: until another command) |
| `SetVelocity(vx, vy, vyaw, duration)` | The same, for `duration` seconds |
| `StopMove()` | Velocity 0 |
| `Start()` | The walking state (FSM 811), the initial one |
| `Damp()`, `Stance()`, `ZeroTorque()`... | Other states: the robot stops and does not walk until `Start()` |

**Head, waist and arms:** publish `unitree_hg LowCmd_` on `rt/arm_sdk`, as the `r1_arm_sdk_dds_example`
of unitree_sdk2 does: `mode_pr` is the weight (100: the SDK has the joints; 0: the robot's rest
pose), and `motor_cmd[i].q` the position of motor `i`:

| Motors | Index |
|---|---|
| Left arm: shoulder pitch, roll, yaw, elbow, wrist roll | 15-19 |
| Right arm | 22-26 |
| Waist yaw | 13 |
| Head pitch, head yaw | 29, 30 |

The gains (`kp`, `kd`) are ignored: the joints reach the position at their largest speed.

**State:** `rt/lowstate` (`unitree_hg LowState_`: `motor_state[i].q`, `.dq`, `imu_state.rpy`) and
`rt/odommodestate` (`unitree_go SportModeState_`: `position`, `velocity`, `yaw_speed`,
`imu_state.rpy`), at 100 Hz.

## What it publishes in ROS

| Topic / frame | Type | From |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `r1_state_publisher` (from `rt/lowstate`), with the names of the URDF |
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | `r1_state_publisher` (from the odometry) |
| TF of the robot (`base_link → pelvis_link → ... → camera_link`) | | `robot_state_publisher`, with `model.description_urdf()` |
| `/r1/camera/image`, `/r1/camera/depth_image` (32FC1, m), `/r1/camera/camera_info` | `sensor_msgs` | Gazebo, stamped with `camera_link` (x forward, not an optical frame) |
| `/clock` | | Gazebo |

`base_link` is on the floor under the pelvis.

## On the real robot

`r1_state_publisher` is not specific to the simulation: on the robot, with its network interface
and domain 0, it gives the same `/joint_states`, `/odom` and TF from the robot's own DDS:

```sh
ros2 run unitree_r1_gazebo r1_state_publisher --ros-args -p interface:=eth0 -p domain:=0
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(python3 -c 'from unitree_r1_gazebo import model; print(model.description_urdf())')"
```

**Not yet tried on the robot.** To check there: the topic of the odometry of the R1
(`rt/odommodestate` on the G1; `odometry_topic`), that `LocoClient.Move` is accepted (an
[issue of Unitree](https://github.com/unitreerobotics/xr_teleoperate/issues/319) reports
`SetVelocity` refused on some R1 configurations), and the camera of the head.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `ModuleNotFoundError: unitree_sdk2py` or `cyclonedds` | The SDK is not installed in the Python that runs the nodes ([Installation](#installation)) |
| `DDS_RETCODE_BAD_PARAMETER` creating a topic | The binding of CycloneDDS does not match the CycloneDDS library loaded (another version, or not built with `--no-build-isolation` against ROS's) |
| The SDK calls time out | Your program and the simulation use another domain or interface (`sdk_domain`, `sdk_interface`; the default is 1 on `lo`) |
| The robot does not walk | Not in the walking state: `loco.Start()` |
| The first launch takes long | It downloads the model of the R1 and the models of the world |

## License

Apache-2.0 (this package). The model of the R1 and the R1 locomotion client are Unitree's
(BSD-3-Clause, `LICENSE.unitree`). Author: Rodrigo Pérez-Rodríguez (Universidad Rey Juan Carlos).
