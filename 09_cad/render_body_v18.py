"""Standalone engineering previews from saved v18 assembly meshes."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v18'
fig=plt.figure(figsize=(16,8))
names=['stand_baseline','stand_narrow','stand_electronics_repack','kneeling_contact_probe']
titles=['Original leg packaging','Narrower legs','Electronics repositioned','Gauntlet + knee pose probe']
for i,(name,title) in enumerate(zip(names,titles)):
    rows=json.loads((OUT/(name+'_mesh.json')).read_text());ax=fig.add_subplot(1,4,i+1,projection='3d');ax.set_proj_type('ortho')
    for r in rows:
        pts=np.asarray(r['points']);tris=pts[np.asarray(r['triangles'])]
        color='#477bac' if r['role']=='motor' else '#b167a4' if r['role']=='electronics' else '#d1a434' if r['role']=='shell' else '#7e8892'
        alpha=.5 if r['role']=='shell' else .85
        ax.add_collection3d(Poly3DCollection(tris,facecolor=color,edgecolor='none',alpha=alpha))
    ground=-2 if name.startswith('stand') else 0
    ax.plot([-170,220,220,-170,-170],[-150,-150,150,150,-150],[ground]*5,color='#60945f',linewidth=.8)
    ax.set(xlim=(-190,230),ylim=(-170,170),zlim=(-5,410));ax.set_box_aspect((420,340,415),zoom=1.25)
    ax.view_init(elev=12,azim=-65);ax.set_axis_off();ax.set_title(title,fontsize=10)
fig.suptitle('v18 integrated 28-DOF study — unresolved collisions retained',fontsize=14)
fig.text(.07,.08,'Blue: motor envelopes  |  purple: electronics  |  gold: Wukong head/crown occupancy (appearance not approved)',fontsize=10)
fig.text(.07,.04,'Actual CAD retains vendor geometry; preview uses motor/electronics boxes. Standing datum: sole pad bottom Z=-2 mm; recovery datum: Z=0.',fontsize=9)
fig.subplots_adjust(top=.89,bottom=.12,wspace=.04);fig.savefig(OUT/'body_comparison.png',dpi=140);plt.close(fig)
print('Saved body_comparison.png')
