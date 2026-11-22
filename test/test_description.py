from pathlib import Path
import xml.etree.ElementTree as ET
import math
import xacro


def robot():
    path = Path(__file__).resolve().parents[1] / 'urdf/runa.urdf.xacro'
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def test_one_connected_tree_and_valid_mesh_references():
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
    for mesh in root.findall('.//mesh'):
        assert (Path(__file__).resolve().parents[1] / mesh.get('filename').removeprefix('package://runa_description/')).is_file()
    for inertial in root.findall('link/inertial'):
        assert float(inertial.find('mass').get('value')) > 0
        assert all(float(inertial.find('inertia').get(k)) > 0 for k in ['ixx','iyy','izz'])


def test_actuators_limits_and_no_hardware():
    root = robot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    for name in ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5','gripper_joint','head_pan_joint','head_tilt_joint','lift_joint']:
        j = joints[name]
        assert j.get('type') in ['revolute','prismatic']
        limits = j.find('limit')
        assert float(limits.get('lower')) <= 0 <= float(limits.get('upper'))
        assert float(limits.get('lower')) < float(limits.get('upper'))
    assert joints['right_finger_joint'].find('mimic').get('joint') == 'gripper_joint'
    assert root.find('ros2_control') is None
    assert root.find('.//plugin') is None
    assert root.findall('.//mesh')


def test_physics_preserves_cad_visuals_and_transforms():
    root=Path(__file__).resolve().parents[1]
    cad=ET.parse(root/'urdf/runa_cad.urdf').getroot()
    physics=ET.parse(root/'urdf/runa_physics.urdf').getroot()
    links={n.get('name'):n for n in physics.findall('link')}
    for n in cad.findall('link'):
        assert [ET.tostring(v) for v in n.findall('visual')] == [ET.tostring(v) for v in links[n.get('name')].findall('visual')]
    joints={j.find('child').get('link'):j for j in physics.findall('joint')}
    for j in cad.findall('joint'):
        other=joints[j.find('child').get('link')]
        for tag in ['parent','child','origin','axis']:
            a,b=j.find(tag),other.find(tag)
            assert (a.attrib if a is not None else None)==(b.attrib if b is not None else None)
