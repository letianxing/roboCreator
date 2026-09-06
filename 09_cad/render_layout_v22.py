"""Scientific CAD overview and old/new local support-margin comparison."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v22'
names=['stand_candidate','lift_h114','lift_h119'];titles=['Standing regression','Local body height114mm','Local body height119mm']
fig=plt.figure(figsize=(15,6))
for i,(name,title) in enumerate(zip(names,titles)):
    rows=json.loads((OUT/(name+'_mesh.json')).read_text());ax=fig.add_subplot(1,3,i+1,projection='3d');ax.set_proj_type('ortho')
    for r in rows:
        p=np.array(r['points']);tris=p[np.array(r['triangles'])];color='#985ab7' if r['name'] in ('battery','power','computer') else '#d8a332' if r['role']=='shell' else '#4384ad' if r['role']=='motor' else '#d69230' if r['role']=='pad' else '#8c949b'
        ax.add_collection3d(Poly3DCollection(tris,facecolor=color,edgecolor='none',alpha=.85))
    ax.set(xlim=(-210,220),ylim=(-210,210),zlim=(-5,410));ax.set_box_aspect((430,420,415));ax.view_init(elev=18,azim=-65);ax.set_axis_off();ax.set_title(title)
fig.suptitle('v22 electronics motion packaging — local sampled geometry only')
fig.text(.05,.055,'Purple: relocated modules. Blue: motor preview bounds. Gold: provisional head/crown. Mounts, cables, dynamics and final Wukong appearance are NOT validated.',fontsize=9)
fig.subplots_adjust(top=.9,bottom=.12,wspace=0);fig.savefig(OUT/'layout_review.png',dpi=150);plt.close(fig)
data=json.loads((OUT/'motion_validation.json').read_text());new=data['samples'];prior=json.loads((ROOT/'05_dynamics/support_v21/contact_margins.json').read_text())['cases'];prior=[r for r in prior if r['scene']=='local_lift' and 114<=r['h_mm']<=120]
fig,ax=plt.subplots(figsize=(9,5))
ax.plot([r['h_mm'] for r in new],[r['support']['support_polygon_margin_mm'] for r in new],'-o',label='v22: sampled geometry and vertical balance pass')
ax.plot([r['h_mm'] for r in prior],[r['support_polygon_margin_mm'] for r in prior],'--x',label='v21: geometry failed below120mm')
ax.axhline(0,color='black',lw=1);ax.set(xlabel='Body reference height / mm',ylabel='COM projection margin / mm',title='Relocation clears a local motion interval; high-end margin is still small')
ax.grid(alpha=.25);ax.legend(fontsize=9);fig.tight_layout();fig.savefig(OUT/'support_comparison.png',dpi=150);plt.close(fig)
