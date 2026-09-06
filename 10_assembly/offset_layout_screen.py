"""Preliminary mounting-offset IK and oriented-box clearance audit.
Not vendor CAD, a manufacturable assembly, or a dynamics model.
"""
import importlib.util
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('arm',ROOT/'04_kinematics/arm_layout_screen.py')
arm=importlib.util.module_from_spec(s);s.loader.exec_module(arm)

LAYOUTS={
 'coaxial_baseline':[[0,0,0],[0,0,0]],
 'lateral_then_down':[[0,.055,0],[0,0,-.055]],
 'down_then_lateral':[[0,0,-.055],[0,.055,0]],
 'lateral_then_forward':[[0,.055,0],[.055,0,0]]
}

def envelope(name,origin,r,axis,small=False):
    i='xyz'.index(axis);a=np.eye(3)[i]
    # Conservative cuboid around AK40 cylinder; small part uses orientation-independent cube.
    dims=np.full(3,.034 if small else .053)
    length=.034 if small else .0402
    dims[i]=length
    # Origin treated as output-face point; 5mm face-to-joint gap is a mounting assumption.
    center=origin-r@a*(length/2+.005)
    return dict(name=name,center=center,rotation=r.copy(),half=dims/2)

def overlap(a,b,padding=.001):
    # SAT on 3+3+9 axes, each box padded 1mm => nominal pair clearance 2mm.
    axes=[a['rotation'][:,i] for i in range(3)]+[b['rotation'][:,i] for i in range(3)]
    axes += [np.cross(a['rotation'][:,i],b['rotation'][:,j]) for i in range(3) for j in range(3)]
    delta=b['center']-a['center']
    for axis in axes:
        n=np.linalg.norm(axis)
        if n<1e-9:continue
        axis=axis/n
        ra=np.sum((a['half']+padding)*np.abs(a['rotation'].T@axis))
        rb=np.sum((b['half']+padding)*np.abs(b['rotation'].T@axis))
        if abs(delta@axis)>ra+rb:return False
    return True

def conflicts(boxes):
    return [[a['name'],b['name']] for a,b in itertools.combinations(boxes,2) if overlap(a,b)]

def fk(q,offsets,shoulder_z=.295):
    p=np.array([0.,.11,shoulder_z]);r=np.eye(3);boxes=[];origins=[]
    steps=[[0,0,0],offsets[0],offsets[1],[0,0,-.08],[0,0,-.08]]
    for i,(axis,step) in enumerate(zip('yxzyx',steps)):
        p=p+r@np.array(step)
        origins.append(p.copy())
        boxes.append(envelope(f'joint_{i}',p,r,axis,small=i==4))
        r=r@arm.rot(axis,q[i])
    direction=r@np.array([0.,0.,-1.])
    tip=p+.04*direction
    return tip,direction,boxes,origins

def solve(offsets,target,direction,z):
    rng=np.random.default_rng(77)
    lo=np.deg2rad([-160,-120,-160,0,-120]);hi=np.deg2rad([160,120,160,150,120])
    def residual(q):
        p,d,_,_=fk(q,offsets,z)
        return np.r_[p-target,.08*(d-direction)]
    solutions=[]
    for seed in range(10):
        q=np.clip(np.zeros(5) if seed==0 else rng.uniform(lo,hi),lo,hi)
        for _ in range(140):
            e=residual(q)
            j=np.column_stack([(residual(q+np.eye(5)[i]*1e-5)-e)/1e-5 for i in range(5)])
            dq=-np.linalg.solve(j.T@j+1e-5*np.eye(5),j.T@e)
            q=np.clip(q+np.clip(dq,-.18,.18),lo,hi)
            if np.linalg.norm(e)<1e-5:break
        p,d,b,_=fk(q,offsets,z)
        pe=float(np.linalg.norm(p-target));de=float(np.degrees(np.arccos(np.clip(d@direction,-1,1))))
        pairs=conflicts(b);passed=pe<=.002 and de<=5
        solutions.append(dict(q_deg=np.degrees(q).tolist(),position_error_mm=pe*1000,
            direction_error_deg=de,pose_pass=passed,box_conflicts=pairs,
            sampled_endpoint_clear=passed and not pairs))
    return min(solutions,key=lambda x:(not x['sampled_endpoint_clear'],not x['pose_pass'],
               len(x['box_conflicts']) if x['pose_pass'] else 0,x['position_error_mm']+.8*x['direction_error_deg']))

def encode(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.bool_):return bool(x)
    raise TypeError(type(x))

def export_urdf(name,boxes):
    # Fixed-link box proxy only; deliberately no inertial/dynamic claims.
    robot=ET.Element('robot',name=name+'_envelope_proxy')
    ET.SubElement(robot,'link',name='world')
    for i,b in enumerate(boxes):
        link=ET.SubElement(robot,'link',name=b['name'])
        visual=ET.SubElement(link,'visual');geo=ET.SubElement(visual,'geometry')
        ET.SubElement(geo,'box',size=' '.join(str(v) for v in 2*b['half']))
        material=ET.SubElement(visual,'material',name=f'm{i}')
        ET.SubElement(material,'color',rgba='0.25 0.55 0.85 0.7')
        j=ET.SubElement(robot,'joint',name=f'fixed_{i}',type='fixed')
        ET.SubElement(j,'parent',link='world');ET.SubElement(j,'child',link=b['name'])
        # Exported neutral poses have identity stator rotations.
        assert np.allclose(b['rotation'],np.eye(3))
        ET.SubElement(j,'origin',xyz=' '.join(str(v) for v in b['center']),rpy='0 0 0')
    ET.indent(robot)
    ET.ElementTree(robot).write(ROOT/'10_assembly'/f'{name}_neutral_proxy.urdf',encoding='utf-8',xml_declaration=True)

def main():
    result={'status':'mounting_proxy_not_manufacturing','shoulder':{},'hip':{}}
    tasks=[('pickup_lowered',[.10,.04,.13],[0,-1,0],.255),('handover',[.15,.04,.24],[0,-1,0],.295)]
    for name,offsets in LAYOUTS.items():
        _,_,b,_=fk(np.zeros(5),offsets)
        rows=[]
        for task,p,d,z in tasks:
            row=solve(offsets,np.array(p),np.array(d),z);row['task']=task;rows.append(row)
        result['shoulder'][name]={'offsets_m':offsets,'neutral_boxes':b,
            'neutral_conflicts':conflicts(b),'tasks':rows}
        export_urdf('shoulder_'+name,b)
    for name,offsets in {'baseline':[[0,0,0],[0,0,0]],
                          'lateral_forward':[[0,.055,0],[.055,0,0]],
                          'forward_lateral':[[.055,0,0],[0,.055,0]]}.items():
        p=np.array([0.,.055,.205]);b=[]
        for i,axis in enumerate('zxy'):
            if i:p=p+np.array(offsets[i-1])
            b.append(envelope(f'hip_{i}',p,np.eye(3),axis))
        result['hip'][name]={'offsets_m':offsets,'neutral_boxes':b,'neutral_conflicts':conflicts(b),
                            'terminal_joint_xyz_m':p.tolist()}
        export_urdf('hip_'+name,b)
    result['limitations']=['AK40 outer cuboid from published 53x40.2mm; exact CAD not loaded',
      'Small actuator uses conservative 34mm cube; mounting axis not manufacturer-verified',
      '5mm shaft-face clearance and all offsets assumed; not hole-pattern design',
      'Only motor-motor box checks; torso, brackets, opposite arm, wiring and bottle omitted',
      'Endpoint tests only; no swept-path guarantee; hip only neutral, no leg IK',
      'Numeric failures are not impossibility proofs; box overlap can be conservative false positive',
      'URDF files are fixed neutral envelope views, NOT the trainable robot URDF']
    path=ROOT/'10_assembly/offset_layout_results_v04.json'
    path.write_text(json.dumps(result,default=encode,indent=2)+'\n')
    for kind in ('shoulder','hip'):
        for name,r in result[kind].items():
            print(kind,name,'neutral_conflicts',len(r['neutral_conflicts']))
            for t in r.get('tasks',[]):
                print(' ',t['task'],'pos_mm',round(t['position_error_mm'],2),'dir_deg',round(t['direction_error_deg'],2),
                      'conflicts',len(t['box_conflicts']),'endpoint_clear',t['sampled_endpoint_clear'])

if __name__=='__main__':main()
