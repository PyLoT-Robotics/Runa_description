from pathlib import Path
import xml.etree.ElementTree as ET
import math
import xacro


def robot():
    path = Path(__file__).resolve().parents[1] / 'urdf/runa.urdf.xacro'
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def test_one_connected_tree_and_positive_inertias():
    root = robot()
    links = {e.get('name') for e in root.findall('link')}
    parents = {j.find('child').get('link'): j.find('parent').get('link') for j in root.findall('joint')}
    assert links - parents.keys() == {'base_footprint'}
    assert len(parents) == len(root.findall('joint')) == len(links)-1
    for name in links:
        seen = set()
        while name in parents:
            assert name not in seen
            seen.add(name)
            name = parents[name]
        assert name == 'base_footprint'
    for inertial in root.findall('link/inertial'):
        assert float(inertial.find('mass').get('value')) > 0
        assert all(float(inertial.find('inertia').get(k)) > 0 for k in ['ixx','iyy','izz'])


def test_actuators_limits_and_no_hardware():
    root = robot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    for name in ['joint1','joint2','joint3','joint4','joint5','gripper_left_joint','base_to_pan_joint','pan_to_tilt_joint','lift_joint']:
        j = joints[name]
        assert j.get('type') in ['revolute','prismatic']
        limits = j.find('limit')
        assert float(limits.get('lower')) <= 0 <= float(limits.get('upper'))
        assert float(limits.get('lower')) < float(limits.get('upper'))
    assert joints['gripper_right_joint'].find('mimic').get('joint') == 'gripper_left_joint'
    assert root.find('ros2_control') is None
    assert root.find('.//plugin') is None
    assert not root.findall('.//mesh')
