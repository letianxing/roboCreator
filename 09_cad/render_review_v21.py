"""Static scientific CAD comparison; no approved exterior styling implied."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v21';info=json.loads((OUT/'review_exports.json').read_text());names=info['names']
fig=plt.figure(figsize=(12,6));n=max(1,len(names))
for i,name in enumerate(names):
    rows=json.loads((OUT/(name+'_mesh.json')).read_text());ax=fig.add_subplot(1,n,i+1,projection='3d');ax.set_proj_type('ortho')
    bad=set();ep=OUT/(name+'_evidence.json')
    if ep.exists():
        for c in json.loads(ep.read_text())['all_collisions']:bad.update([c['a'],c['b']])
    for r in rows:
        pts=np.array(r['points']);tri=pts[np.array(r['triangles'])]
        color='#ce3535' if r['name'] in bad else '#d6a12b' if r['role']=='shell' else '#4788b6' if r['role']=='motor' else '#9468a4' if r['role']=='electronics' else '#dd9a36' if r['role']=='pad' else '#89929b'
        ax.add_collection3d(Poly3DCollection(tri,facecolor=color,edgecolor='none',alpha=.8))
    ax.plot([-200,200,200,-200,-200],[-200,-200,200,200,-200],[0]*5,color='green')
    ax.set(xlim=(-210,190),ylim=(-210,210),zlim=(-5,360));ax.set_box_aspect((400,420,365));ax.view_init(elev=18,azim=-65);ax.set_axis_off();ax.set_title(name+(' | static geometry only' if info['has_static_pass'] else ' | collisions retained'))
fig.suptitle('v21 recovery geometry research — full recovery NOT validated')
fig.text(.06,.06,'Blue: motor bounds for preview; gold: provisional head/crown, NOT approved Wukong styling. CAD retains exact modeled shapes.',fontsize=8)
fig.tight_layout(rect=(0,.1,1,.94));fig.savefig(OUT/'review.png',dpi=150);plt.close(fig)
print('Saved review.png')
