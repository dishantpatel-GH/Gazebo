"""Build a Gazebo-ready URDF for the Unitree G1 (29 DoF, mode 15) with Dex1-1 grippers.

The Unitree URDF in robots/g1_description (unmodified copy from unitree_ros) is patched in memory:
  * relative mesh paths are turned into absolute file:// URIs,
  * the MuJoCo-only <mujoco> block is removed,
  * the pelvis is welded to the world at standing height (no balance controller needed),
  * a <ros2_control> block and the gz_ros2_control plugin are added,
  * contact friction is raised on the gripper fingers and feet,
  * a RealSense D435i (RGB-D + IMU) is simulated on the head mount `d435_link`,
  * a RealSense D405 (RGB-D) is added on top of each Dex1-1 gripper.
"""

import math
import os
import xml.etree.ElementTree as ET

URDF_RELPATH = os.path.join('robots', 'g1_description', 'g1_29dof_mode_15_with_dex1_1.urdf')
CONTROLLERS_RELPATH = os.path.join('config', 'g1_controllers.yaml')

# Pelvis height with straight legs: soles are 0.7919 m below the pelvis origin.
PELVIS_HEIGHT = 0.793

LEG_JOINTS = [
    f'{side}_{name}_joint'
    for side in ('left', 'right')
    for name in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
]
WAIST_JOINTS = ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
ARM_JOINT_NAMES = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                   'wrist_roll', 'wrist_pitch', 'wrist_yaw')
LEFT_ARM_JOINTS = [f'left_{name}_joint' for name in ARM_JOINT_NAMES]
RIGHT_ARM_JOINTS = [f'right_{name}_joint' for name in ARM_JOINT_NAMES]
# Dex1-1 prismatic fingers: +0.0245 = fully open, -0.02 = fully closed.
LEFT_GRIPPER_JOINTS = ['left_dex1_finger_joint_1', 'left_dex1_finger_joint_2']
RIGHT_GRIPPER_JOINTS = ['right_dex1_finger_joint_1', 'right_dex1_finger_joint_2']

ALL_JOINTS = (LEG_JOINTS + WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
              + LEFT_GRIPPER_JOINTS + RIGHT_GRIPPER_JOINTS)

# Start with forearms pointing forward over the table and grippers open.
INITIAL_POSITIONS = {j: 0.0245 for j in LEFT_GRIPPER_JOINTS + RIGHT_GRIPPER_JOINTS}

# D405 wrist camera (42 x 42 x 23 mm) on top of each Dex1-1 base, tilted down so the
# fingers and the area just past the fingertips are in view.
D405_SIZE = (0.023, 0.042, 0.042)
D405_ORIGIN = {'xyz': '0.058 0 0.055', 'rpy': '0 0.45 0'}

# ROS optical frame (z forward, x right, y down) relative to a gz camera frame (x forward, z up).
OPTICAL_RPY = f'{-math.pi / 2} 0 {-math.pi / 2}'

HIGH_FRICTION_LINKS = [
    'left_dex1_finger_link_1', 'left_dex1_finger_link_2',
    'right_dex1_finger_link_1', 'right_dex1_finger_link_2',
    'left_ankle_roll_link', 'right_ankle_roll_link',
]


def _absolutize_meshes(robot, base_dir):
    for mesh in robot.iter('mesh'):
        filename = mesh.get('filename', '')
        if '://' not in filename:
            path = filename if os.path.isabs(filename) else os.path.join(base_dir, filename)
            mesh.set('filename', 'file://' + os.path.abspath(path))


def _add_fixed_base(robot):
    ET.SubElement(robot, 'link', name='world')
    joint = ET.SubElement(robot, 'joint', name='world_to_pelvis', type='fixed')
    ET.SubElement(joint, 'origin', xyz=f'0 0 {PELVIS_HEIGHT}', rpy='0 0 0')
    ET.SubElement(joint, 'parent', link='world')
    ET.SubElement(joint, 'child', link='pelvis')


def _add_ros2_control(robot, joint_limits, initial_positions):
    control = ET.SubElement(robot, 'ros2_control', name='GazeboSimSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for name in ALL_JOINTS:
        lower, upper = joint_limits[name]
        joint = ET.SubElement(control, 'joint', name=name)
        cmd = ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(cmd, 'param', name='min').text = str(lower)
        ET.SubElement(cmd, 'param', name='max').text = str(upper)
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(
            initial_positions.get(name, 0.0))
        ET.SubElement(joint, 'state_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='effort')


def _add_gazebo_tags(robot, controllers_yaml):
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='gz_ros2_control-system',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = controllers_yaml
    # Velocity gain used by gz_ros2_control to track position commands.
    ET.SubElement(plugin, 'position_proportional_gain').text = '0.5'

    for link in HIGH_FRICTION_LINKS:
        ref = ET.SubElement(robot, 'gazebo', reference=link)
        ET.SubElement(ref, 'mu1').text = '1.5'
        ET.SubElement(ref, 'mu2').text = '1.5'


def _add_fixed_link(robot, name, parent, xyz='0 0 0', rpy='0 0 0'):
    link = ET.SubElement(robot, 'link', name=name)
    joint = ET.SubElement(robot, 'joint', name=f'{name}_joint', type='fixed')
    ET.SubElement(joint, 'origin', xyz=xyz, rpy=rpy)
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=name)
    return link


def _rgbd_sensor(parent, name, frame_id, hfov, depth_near, depth_far, pose='0 0 0 0 0 0'):
    sensor = ET.SubElement(parent, 'sensor', name=name, type='rgbd_camera')
    ET.SubElement(sensor, 'pose').text = pose
    ET.SubElement(sensor, 'always_on').text = '1'
    ET.SubElement(sensor, 'update_rate').text = '30'
    ET.SubElement(sensor, 'visualize').text = 'false'
    ET.SubElement(sensor, 'topic').text = name
    ET.SubElement(sensor, 'gz_frame_id').text = frame_id
    camera = ET.SubElement(sensor, 'camera', name=name)
    ET.SubElement(camera, 'horizontal_fov').text = str(hfov)
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = '640'
    ET.SubElement(image, 'height').text = '480'
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.01'
    ET.SubElement(clip, 'far').text = str(depth_far)
    depth_clip = ET.SubElement(ET.SubElement(camera, 'depth_camera'), 'clip')
    ET.SubElement(depth_clip, 'near').text = str(depth_near)
    ET.SubElement(depth_clip, 'far').text = str(depth_far)


def _add_cameras(robot):
    # Head: D435i on the stock G1 mount. Color FOV 69.4 deg, depth from 0.1 m.
    _add_fixed_link(robot, 'head_camera_optical_frame', 'd435_link', rpy=OPTICAL_RPY)
    head = ET.SubElement(robot, 'gazebo', reference='d435_link')
    _rgbd_sensor(head, 'head_camera', 'head_camera_optical_frame',
                 hfov=1.2112, depth_near=0.1, depth_far=10.0)
    imu = ET.SubElement(head, 'sensor', name='head_camera_imu', type='imu')
    ET.SubElement(imu, 'always_on').text = '1'
    ET.SubElement(imu, 'update_rate').text = '200'
    ET.SubElement(imu, 'topic').text = 'head_camera/imu'
    ET.SubElement(imu, 'gz_frame_id').text = 'd435_link'

    # Wrists: D405, 87 deg FOV, depth 7 cm - 1 m (best below 0.5 m).
    for side in ('left', 'right'):
        name = f'{side}_wrist_camera'
        link = _add_fixed_link(robot, f'{side}_wrist_d405_link', f'{side}_dex1_base_link',
                               **D405_ORIGIN)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.06')
        ET.SubElement(inertial, 'inertia', ixx='1.8e-5', iyy='1.1e-5', izz='1.1e-5',
                      ixy='0', ixz='0', iyz='0')
        for tag in ('visual', 'collision'):
            element = ET.SubElement(link, tag)
            geometry = ET.SubElement(element, 'geometry')
            ET.SubElement(geometry, 'box', size=' '.join(str(s) for s in D405_SIZE))
            if tag == 'visual':
                material = ET.SubElement(element, 'material', name='d405_silver')
                ET.SubElement(material, 'color', rgba='0.55 0.56 0.58 1')
        _add_fixed_link(robot, f'{name}_optical_frame', f'{side}_wrist_d405_link',
                        rpy=OPTICAL_RPY)
        ref = ET.SubElement(robot, 'gazebo', reference=f'{side}_wrist_d405_link')
        # Sensor sits on the front face so the camera body does not block the view.
        _rgbd_sensor(ref, name, f'{name}_optical_frame', hfov=1.5184, depth_near=0.07,
                     depth_far=1.0, pose=f'{D405_SIZE[0] / 2 + 0.001} 0 0 0 0 0')


def build_robot_description(pkg_share):
    """Return the URDF string; pkg_share is the installed g1_gazebo share directory."""
    urdf_path = os.path.join(pkg_share, URDF_RELPATH)
    robot = ET.parse(urdf_path).getroot()

    for mujoco in robot.findall('mujoco'):
        robot.remove(mujoco)
    _absolutize_meshes(robot, os.path.dirname(urdf_path))

    joint_limits = {}
    for joint in robot.findall('joint'):
        limit = joint.find('limit')
        if limit is not None:
            joint_limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))

    _add_fixed_base(robot)
    _add_cameras(robot)
    _add_ros2_control(robot, joint_limits, INITIAL_POSITIONS)
    _add_gazebo_tags(robot, os.path.join(pkg_share, CONTROLLERS_RELPATH))

    ET.indent(robot)
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding='unicode')
