"""Scientific kinematic assembly views; not final Wukong concept approval."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/neck_hardware_v28_final'
fig=plt.figure(figsize=(14,7))
for i,name in enumerate(['compact_cable_y0_p0','compact_cable_y0_p50','compact_cable_y30_p45']):
    rows=json.loads((OUT/(name+'_mesh.json')).read_text());ax=fig.add_subplot(1,3,i+1,projection='3d')
    for r in rows:
        n=r['name'];p=np.array(r['points']);faces=p[np.array(r['triangles'])]
        col='#c99528' if n in ('WukongHeadShell','CrownOccupancy') else '#cf5260' if n=='camera' else '#3079ad' if n.startswith('NeckMotor') else '#32ac68' if n=='NeckCable_PROBE' else '#9b9ea3'
        alpha=.85 if n in ('WukongHeadShell','CrownOccupancy','camera','NeckMotor0','NeckMotor1','NeckCable_PROBE') else .22
        ax.add_collection3d(Poly3DCollection(faces,facecolor=col,edgecolor='none',alpha=alpha))
    ax.set(xlim=(-130,140),ylim=(-150,150),zlim=(210,440));ax.set_box_aspect((270,300,230));ax.view_init(elev=20,azim=-52);ax.set_axis_off();ax.set_title(name)
fig.suptitle('v28 proposed yaw-pitch chain + shared X330 reference CAD + free-end cable proxy')
fig.text(.04,.07,'Gold: head/crown. Red: camera envelope. Blue: motor bounds in preview (native CAD retains vendor solids). Green: FAILED 38mm flex-segment probe.',fontsize=9)
fig.text(.04,.035,'Some combined yaw/pitch poses collide. Brackets, rotor attachment, true USB cable and final Wukong appearance are NOT validated.',fontsize=9)
fig.subplots_adjust(top=.9,bottom=.12,wspace=0);fig.savefig(OUT/'neck_review.png',dpi=150);plt.close(fig)
