"""Reproducible envelope sampling and artifact sanity checks."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from offset_layout_screen import fk,conflicts,overlap

ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'offset_layout_results_v04.json').read_text())
a=r['shoulder']['lateral_then_down']
out={'status':'sampled_proxy_checks_only','paths':[]}
for t in a['tasks']:
    z=.255 if t['task']=='pickup_lowered' else .295
    q=np.deg2rad(t['q_deg']);hits=[]
    for i,u in enumerate(np.linspace(0,1,101)):
        _,_,b,_=fk(u*q,a['offsets_m'],z)
        if conflicts(b):hits.append(i)
    out['paths'].append({'task':t['task'],'samples':101,'conflicting_frames':hits,
        'note':'joint-space straight interpolation, not executable time trajectory; full robot omitted'})
for path in ROOT.glob('*_neutral_proxy.urdf'):
    robot=ET.parse(path).getroot()
    links={x.attrib['name'] for x in robot.findall('link')}
    assert len(links)==len(robot.findall('link'))
    for joint in robot.findall('joint'):
        assert joint.find('parent').attrib['link'] in links
        assert joint.find('child').attrib['link'] in links
    assert len(robot.findall('joint'))==len(links)-1
box={'center':np.zeros(3),'rotation':np.eye(3),'half':np.ones(3)*.02}
assert overlap(box,box)
far={**box,'center':np.array([1.,0,0])}
assert not overlap(box,far)
out['hip_geometry']={'required_leg_reach_m':float(np.linalg.norm([.045,.075,.18])),
    'available_leg_reach_m':.18,'max_vertical_reach_m':float(np.sqrt(.18**2-.045**2-.075**2)),
    'conditions':'neutral best hip candidate with original ankle positions; excludes sole offset in reach'}
out['urdf_sanity']='XML names and parent-child references passed, not simulator load test'
(ROOT/'offset_verification_v04.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
