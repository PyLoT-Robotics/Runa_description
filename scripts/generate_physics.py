#!/usr/bin/env python3
"""Regenerate approximate simulation physics from the reviewed CAD URDF.

Run from anywhere after installing NumPy. The reviewed CAD source is untouched.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
P=Path(__file__).resolve().parents[1]
r=ET.fromstring((P/'urdf/runa_cad.urdf').read_text())
aliases={**{f'arm_joint_{i}':f'joint{i}' for i in range(1,6)},'gripper_joint':'gripper_left_joint','right_finger_joint':'gripper_right_joint','head_pan_joint':'base_to_pan_joint','head_tilt_joint':'pan_to_tilt_joint'}
def points(filename):
 data=(P/'meshes'/filename).read_bytes();n=int.from_bytes(data[80:84],'little')
 if len(data)==84+n*50:
  dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]);return np.frombuffer(data,offset=84,count=n,dtype=dtype)['vertices'].reshape(-1,3).astype(float)
 vals=[list(map(float,line.split()[1:])) for line in data.decode().splitlines() if line.strip().startswith('vertex')]
 return np.array(vals)
for joint in r.findall('joint'):
 if joint.get('name') in aliases:joint.set('name',aliases[joint.get('name')])
 mimic=joint.find('mimic')
 if mimic is not None:mimic.set('joint',aliases.get(mimic.get('joint'),mimic.get('joint')))
 limit=joint.find('limit')
 if limit is not None:
  limit.set('effort','1000' if joint.get('name')=='lift_joint' else '30')
  limit.set('velocity','0.1' if joint.get('type')=='prismatic' else '2.0')
 if joint.get('type') in ['revolute','prismatic','continuous']:
  ET.SubElement(joint,'dynamics',{'damping':'0.1','friction':'0.01'})
for link in r.findall('link'):
 name=link.get('name'); meshes=link.findall('visual/geometry/mesh')
 if not meshes:continue
 cloud=np.concatenate([points(m.get('filename').split('/meshes/')[1]) for m in meshes]);lo=cloud.min(axis=0);hi=cloud.max(axis=0);size=np.maximum(hi-lo,0.005);center=(lo+hi)/2
 wheel=name in ['left_wheel_link','right_wheel_link'];omni='omni_link' in name
 mass=18 if name=='base_link' else (0.4 if wheel else (0.1 if omni else (0.03 if 'finger' in name else 0.3)))
 inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',{'xyz':' '.join(map(str,center))});ET.SubElement(inertial,'mass',{'value':str(mass)})
 xx,yy,zz=size;ET.SubElement(inertial,'inertia',{'ixx':str(mass*(yy*yy+zz*zz)/12),'iyy':str(mass*(xx*xx+zz*zz)/12),'izz':str(mass*(xx*xx+yy*yy)/12),'ixy':'0','ixz':'0','iyz':'0'})
 for collision in list(link.findall('collision')):link.remove(collision)
 c=ET.SubElement(link,'collision',{'name':'simulation_proxy'});g=ET.SubElement(c,'geometry')
 if wheel:
  ET.SubElement(c,'origin',{'rpy':'1.5707963267948966 0 0'});ET.SubElement(g,'cylinder',{'radius':'0.05035','length':str(size[1])})
 elif omni:
  ET.SubElement(g,'sphere',{'radius':'0.05035'})
  gz=ET.SubElement(r,'gazebo',{'reference':name});ET.SubElement(gz,'mu1').text='0.001';ET.SubElement(gz,'mu2').text='0.001'
 else:
  ET.SubElement(c,'origin',{'xyz':' '.join(map(str,center))});ET.SubElement(g,'box',{'size':' '.join(map(str,size))})
# Keep CAD frame names and add sensor/API TF aliases without changing CAD geometry.
for name,parent,xyz,rpy in [
 ('d455_color_optical_frame','head_camera_optical_frame','0 0 0','0 0 0'),
 ('d415_color_optical_frame','wrist_camera_optical_frame','0 0 0','0 0 0'),
 ('camera_link','head_camera_link','0 0 0','0 0 0'),
 ('d415_link','wrist_camera_link','0 0 0','0 0 0'),
 ('tcp','tool0','0 0 0','0 0 0'),
 ('laser','base_link','0.2 0 0.19965','0 0 0'),
 ('bno055_link','base_link','0 0 0','0 0 0'),
 ('livox_frame','base_link','0.2 0 0.09965','-0.21 -0.03 -1.57')]:
 ET.SubElement(r,'link',{'name':name});j=ET.SubElement(r,'joint',{'name':name+'_alias_joint','type':'fixed'});ET.SubElement(j,'parent',{'link':parent});ET.SubElement(j,'child',{'link':name});ET.SubElement(j,'origin',{'xyz':xyz,'rpy':rpy})
ET.indent(r);(P/'urdf/runa_physics.urdf').write_text('<?xml version="1.0"?>\n'+ET.tostring(r,encoding='unicode')+'\n')
