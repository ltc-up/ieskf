"""Bind explicitly conditional historical URDF to recorded TF; private output only."""
import argparse,json,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--historical-candidate',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);a=p.parse_args();r=json.loads(a.raw.read_text());u=ET.fromstring(subprocess.check_output(['xacro',str(a.historical_candidate)],text=True));known={x['child']:x for x in r['/tf_static']};translation=[];rotation=[];missing=[];rotation_mismatch_links=[];reference_rotation=[]
for j in u.findall('joint'):
 if j.get('type')!='fixed':continue
 o=j.find('origin');child=j.find('child').get('link');z=dict(parent=j.find('parent').get('link'),child=child,p=list(map(float,o.get('xyz','0 0 0').split())),q=Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_quat().tolist())
 if child not in known:r['/tf_static'].append(z);missing.append(child)
 else:
  b=known[child]
  if b['parent']!=z['parent']:raise ValueError('RECORDED_PARENT_MISMATCH')
  translation.append(float(np.linalg.norm(np.array(b['p'])-z['p'])));angle=float((Rotation.from_quat(b['q']).inv()*Rotation.from_quat(z['q'])).magnitude());rotation.append(angle)
  if angle>1e-8:rotation_mismatch_links.append(child)
  if child in ['front_wheel','imu_bosch','vectornav']:reference_rotation.append(angle)
a.private.mkdir(parents=True,exist_ok=True);(a.private/'reference_bound.json').write_text(json.dumps(r));v=u.find("link[@name='base_link']/visual");box=v.find('geometry/box')
if box is not None:
 size=np.array(list(map(float,box.get('size').split())));o=v.find('origin');offset=np.array(list(map(float,o.get('xyz','0 0 0').split())));rot=Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_matrix();xyz=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*size/2;xyz=xyz@rot.T+offset
 (a.private/'visual_outline.json').write_text(json.dumps(dict(qualification='HISTORICAL_COPY_VISUAL_BOX_ONLY_NOT_MEASURED_BODY_OR_TYRES',polygons={'visual_body_candidate':[xyz.tolist()]})))
a.audit.write_text(json.dumps(dict(recorded_ant1_tf='vectornav_ant1' in known,known_TF_max_translation_difference_m=max(translation,default=0),known_TF_max_rotation_difference_rad=max(rotation,default=0),recorded_vs_candidate_rotation_mismatch_links=rotation_mismatch_links,reference_chain_max_rotation_difference_rad=max(reference_rotation,default=0),missing_links_from_explicit_candidate=missing,antenna_tf_source='historical project-copy URDF; installation uncalibrated',body_source='historical visual box; not measured body',tyre_geometry_available=False),indent=2)+'\n')
