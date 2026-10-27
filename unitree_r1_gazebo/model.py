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

"""The model of the Unitree R1, from the URDF and meshes of Unitree (unitree_ros,
robots/r1_description, BSD-3-Clause), downloaded once, at a fixed commit, into
~/.cache/unitree_r1_gazebo/r1_description/<commit>. Two URDFs come from it:

    description_urdf()  the robot as it is, rooted at base_link (on the floor under the pelvis),
                        for robot_state_publisher, in simulation and on the real robot
    simulation_urdf()   the robot for Gazebo (below)

The simulated robot has no physics (no gravity, no collisions): what the locomotion controller
of the real robot does is done by three joints under its body, moved with velocities (the
commands of LocoClient.Move):

    base_x_joint, base_y_joint  the body over the floor, in the axes of the world
    body_yaw_joint              the body turning on the spot

while its legs step (an animation of the gait, gait.py: they do not carry the body), and what
the arm SDK moves (rt/arm_sdk) by position controllers: the waist yaw, the head (pitch and yaw,
which carry the camera) and the arms. The head carries an RGB-D camera.

    ros2 run unitree_r1_gazebo fetch_r1_model        # downloads the model (once)
"""

import math
import os
import re
import urllib.request
import xml.etree.ElementTree as ET

from unitree_r1_gazebo.joints import ARM_SDK, BASE_X, BASE_Y, BODY_YAW, LEGS

UNITREE_COMMIT = '1c300a6127dcf6f998cbeddd372f7c0eddf24d41'
UNITREE_URL = ('https://raw.githubusercontent.com/unitreerobotics/unitree_ros/'
               f'{UNITREE_COMMIT}/robots/r1_description')
CACHE = os.path.expanduser(os.path.join('~', '.cache', 'unitree_r1_gazebo', 'r1_description',
                                        UNITREE_COMMIT))

# The joints moved by position, with their largest speed (rad/s)
POSITION_JOINTS = {name: 2.0 for name in ARM_SDK}
POSITION_JOINTS.update({'waist_yaw_joint': 1.0, 'head_pitch_joint': 1.5, 'head_yaw_joint': 2.0})
# The legs follow the gait: faster, with a higher gain
POSITION_JOINTS.update({name: 10.0 for name in LEGS})
GAINS = {name: 20.0 for name in LEGS}
# The joints moved by velocity: the body over the floor (m/s) and turning (rad/s), the limits
# of the locomotion controller of the simulation (sdk_sim)
VELOCITY_JOINTS = {BASE_X: 1.0, BASE_Y: 1.0, BODY_YAW: 2.0}
# Height of the pelvis over the floor, standing with straight legs (the soles of the feet)
PELVIS_HEIGHT = 0.72
# The colours of the parts (Unitree's URDF paints them white or grey): an approximation of the
# robot, light shells and dark joints, matched by the start of the link names
COLOURS = {
    'shell': (0.82, 0.83, 0.85),
    'joint': (0.18, 0.19, 0.21),
    'black': (0.05, 0.05, 0.06),
    'eyes': (0.15, 0.75, 1.0),    # the lights of the eyes, and a stripe on the head
}
# Details that the meshes have in one piece with the head (head_yaw_link): the eyes, a disc on each
# lens of the visor (inside the near clip of the camera: it does not see them), and a stripe along
# the top of the head, in pieces that follow its curve. Positions in the frame of the head (m),
# measured on the mesh: the lenses, and the height of the top along x
EYES = {'radius': 0.008, 'depth': 0.003, 'x': 0.0848, 'y': 0.025, 'z': -0.024}
CREST_PROFILE = [(-0.05, 0.0467), (-0.04, 0.0558), (-0.03, 0.0604), (-0.02, 0.0644), (-0.01, 0.0675),
                 (0.00, 0.0688), (0.01, 0.0689), (0.02, 0.0686), (0.03, 0.0666), (0.04, 0.0631),
                 (0.05, 0.0580), (0.06, 0.0539)]
CREST_WIDTH, CREST_THICKNESS = 0.02, 0.003
PART_COLOURS = [
    ('camera_link', 'black'), ('imu', 'joint'),
    ('left_ankle_roll', 'black'), ('right_ankle_roll', 'black'),
    ('left_ankle', 'joint'), ('right_ankle', 'joint'),
    ('left_hip_pitch', 'joint'), ('right_hip_pitch', 'joint'),
    ('left_hip_roll', 'joint'), ('right_hip_roll', 'joint'),
    ('waist_roll', 'joint'), ('left_shoulder_pitch', 'joint'), ('right_shoulder_pitch', 'joint'),
    ('left_wrist', 'joint'), ('right_wrist', 'joint'),
    ('', 'shell'),
]
# The camera of the head. The real one: to be checked on the robot
CAMERA = {'width': 640, 'height': 480, 'hfov': 1.2, 'rate': 15}


def fetch():
    """The folder with R1.urdf and its meshes, downloaded the first time."""
    urdf = os.path.join(CACHE, 'R1.urdf')
    if not os.path.isfile(urdf):
        os.makedirs(os.path.join(CACHE, 'meshes'), exist_ok=True)
        urllib.request.urlretrieve(f'{UNITREE_URL}/R1.urdf', urdf + '.part')
        os.replace(urdf + '.part', urdf)
    for name in sorted(set(re.findall(r'filename="meshes/([^"]+)"', open(urdf).read()))):
        path = os.path.join(CACHE, 'meshes', name)
        if not os.path.isfile(path):
            urllib.request.urlretrieve(f'{UNITREE_URL}/meshes/{name}', path + '.part')
            os.replace(path + '.part', path)
    return CACHE


def _element(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, {k: str(v) for k, v in attributes.items()})
    if text is not None:
        element.text = str(text)
    return element


def _inertial(link):
    # Some mass: Gazebo drops a massless link fixed to the world
    inertial = _element(link, 'inertial')
    _element(inertial, 'mass', value=1.0)
    _element(inertial, 'inertia', ixx=0.01, ixy=0, ixz=0, iyy=0.01, iyz=0, izz=0.01)


def _robot(model_dir):
    """The URDF of Unitree without its floating base, with absolute mesh paths and base_link
    on the floor under the pelvis."""
    robot = ET.parse(os.path.join(model_dir, 'R1.urdf')).getroot()
    for mesh in robot.iter('mesh'):
        mesh.set('filename', 'file://' + os.path.join(model_dir, mesh.get('filename')))
    for joint in robot.findall('joint'):
        if joint.get('type') == 'floating':
            robot.remove(joint)
    for link in robot.findall('link'):
        if link.get('name') == 'world':
            robot.remove(link)
    for link in robot.findall('link'):
        colour = next(c for prefix, c in PART_COLOURS if link.get('name', '').startswith(prefix))
        for visual in link.findall('visual'):
            for material in visual.findall('material'):
                visual.remove(material)
            material = _element(visual, 'material', name=f'r1_{colour}')
            _element(material, 'color', rgba='{} {} {} 1'.format(*COLOURS[colour]))
    head = next(link for link in robot.findall('link') if link.get('name') == 'head_yaw_link')
    eye = ('cylinder', {'radius': EYES['radius'], 'length': EYES['depth']})
    details = [('left_eye', eye, (EYES['x'], EYES['y'], EYES['z']), (0, math.pi / 2, 0)),
               ('right_eye', eye, (EYES['x'], -EYES['y'], EYES['z']), (0, math.pi / 2, 0))]
    for k, ((x0, z0), (x1, z1)) in enumerate(zip(CREST_PROFILE, CREST_PROFILE[1:])):
        # A piece between two points of the top, tilted as the head there, half sunk into it
        length = math.hypot(x1 - x0, z1 - z0)
        details.append((f'crest_{k}', ('box', {'size': f'{length + 0.002} {CREST_WIDTH} {CREST_THICKNESS}'}),
                        ((x0 + x1) / 2, 0, (z0 + z1) / 2), (0, -math.atan2(z1 - z0, x1 - x0), 0)))
    for name, geometry, xyz, rpy in details:
        visual = _element(head, 'visual', name=name)
        _element(visual, 'origin', xyz='{} {} {}'.format(*xyz), rpy='{} {} {}'.format(*rpy))
        _element(_element(visual, 'geometry'), geometry[0], **geometry[1])
        material = _element(visual, 'material', name='r1_eyes')
        _element(material, 'color', rgba='{} {} {} 1'.format(*COLOURS['eyes']))
    _element(robot, 'link', name='base_link')
    pelvis = _element(robot, 'joint', name='base_to_pelvis_joint', type='fixed')
    _element(pelvis, 'origin', xyz=f'0 0 {PELVIS_HEIGHT}', rpy='0 0 0')
    _element(pelvis, 'parent', link='base_link')
    _element(pelvis, 'child', link='pelvis_link')
    return robot


def description_urdf(model_dir=None):
    """The R1 as it is (every joint), rooted at base_link, as a string."""
    robot = _robot(model_dir or fetch())
    ET.indent(robot)
    return ET.tostring(robot, encoding='unicode')


def simulation_urdf(model_dir=None, name='r1'):
    """The URDF of the simulated R1 for Gazebo, as a string. Its Gazebo topics start with
    `name`: <name>/<joint>/cmd_pos (position joints), <name>/<joint>/cmd_vel (velocity joints),
    <name>/joint_state and <name>/camera."""
    robot = _robot(model_dir or fetch())
    # No collisions: the robot does not touch anything (it is fixed to the world), and the meshes
    # of the whole body would slow the simulation down
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        # The simulated camera sits inside the camera module: its mesh would fill the image
        if link.get('name') == 'camera_link':
            for visual in link.findall('visual'):
                link.remove(visual)
    # Rigid but for the joints that the arm SDK moves
    for joint in robot.findall('joint'):
        if joint.get('type') != 'fixed' and joint.get('name') not in POSITION_JOINTS:
            joint.set('type', 'fixed')
    # Fixed to the world, the body moving over the floor and turning about base_link:
    # world -> odom -> (base_x_joint) -> (base_y_joint) -> (body_yaw_joint) base_link
    for link_name in ('world', 'odom', 'base_x_link', 'base_y_link'):
        link = _element(robot, 'link', name=link_name)
        if link_name != 'world':
            _inertial(link)
    fixed = _element(robot, 'joint', name='world_joint', type='fixed')
    _element(fixed, 'parent', link='world')
    _element(fixed, 'child', link='odom')
    for joint_name, kind, parent, child, axis in (
            (BASE_X, 'prismatic', 'odom', 'base_x_link', '1 0 0'),
            (BASE_Y, 'prismatic', 'base_x_link', 'base_y_link', '0 1 0'),
            (BODY_YAW, 'continuous', 'base_y_link', 'base_link', '0 0 1')):
        joint = _element(robot, 'joint', name=joint_name, type=kind)
        _element(joint, 'parent', link=parent)
        _element(joint, 'child', link=child)
        _element(joint, 'axis', xyz=axis)
        if kind == 'prismatic':
            _element(joint, 'limit', lower=-100.0, upper=100.0, effort=1000.0,
                     velocity=VELOCITY_JOINTS[joint_name])
    # Gazebo: no gravity, the camera and the joints
    for link in robot.findall('link'):
        if link.get('name') != 'world':
            _element(_element(robot, 'gazebo', reference=link.get('name')), 'gravity', 'false')
    # The camera link is kept (not merged into the head), so that its pose is that of the camera
    _element(_element(robot, 'gazebo', reference='camera_joint'), 'preserveFixedJoint', 'true')
    sensor = _element(_element(robot, 'gazebo', reference='camera_link'), 'sensor',
                      name='head_camera', type='rgbd_camera')
    camera = _element(sensor, 'camera')
    _element(camera, 'horizontal_fov', CAMERA['hfov'])
    image = _element(camera, 'image')
    _element(image, 'width', CAMERA['width'])
    _element(image, 'height', CAMERA['height'])
    clip = _element(camera, 'clip')
    _element(clip, 'near', 0.1)
    _element(clip, 'far', 12.0)
    _element(sensor, 'always_on', 1)
    _element(sensor, 'update_rate', CAMERA['rate'])
    _element(sensor, 'topic', f'{name}/camera')
    _element(sensor, 'gz_frame_id', 'camera_link')
    plugins = _element(robot, 'gazebo')
    for joint, speed in POSITION_JOINTS.items():
        # Velocity commands towards the commanded position, at most at the joint's speed: no
        # gains to tune with the inertia of the body
        plugin = _element(plugins, 'plugin', filename='gz-sim-joint-position-controller-system',
                          name='gz::sim::systems::JointPositionController')
        _element(plugin, 'joint_name', joint)
        _element(plugin, 'topic', f'{name}/{joint}/cmd_pos')
        _element(plugin, 'use_velocity_commands', 'true')
        _element(plugin, 'p_gain', GAINS.get(joint, 4.0))
        _element(plugin, 'cmd_max', speed)
        _element(plugin, 'cmd_min', -speed)
    for joint in VELOCITY_JOINTS:
        plugin = _element(plugins, 'plugin', filename='gz-sim-joint-controller-system',
                          name='gz::sim::systems::JointController')
        _element(plugin, 'joint_name', joint)
        _element(plugin, 'topic', f'{name}/{joint}/cmd_vel')
    plugin = _element(plugins, 'plugin', filename='gz-sim-joint-state-publisher-system',
                      name='gz::sim::systems::JointStatePublisher')
    _element(plugin, 'topic', f'{name}/joint_state')
    for joint in (*POSITION_JOINTS, *VELOCITY_JOINTS):
        _element(plugin, 'joint_name', joint)
    ET.indent(robot)
    return ET.tostring(robot, encoding='unicode')


def fetch_main():
    print(f'The model of the R1 is in {fetch()}')
