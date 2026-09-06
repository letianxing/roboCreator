import json,argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v12','v13'],default='v12');args=parser.parse_args();VER=args.variant
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad'/('fullbody_'+VER)

def panel(fig,position,pose,mode,title,elev=18,azim=35):
    ax=fig.add_subplot(*position,projection='3d');ax.set_proj_type('ortho')
    data=json.loads((OUT/pose/'render_mesh.json').read_text())
    for o in data:
        if mode=='mechanical' and o['layer']=='shell':continue
        pts=np.array(o['points']);tris=pts[np.array(o['triangles'])]
        alpha=.20 if o['name']=='Torso_Frame' else .9 if o['layer']=='payload' else 1
        ax.add_collection3d(Poly3DCollection(tris,facecolor=o['color'],edgecolor=o['color'],linewidths=.015,alpha=alpha))
    ax.set(xlim=(-130,240),ylim=(-235,235),zlim=(-5,405));ax.set_box_aspect((370,470,410));ax.view_init(elev=elev,azim=azim)
    ax.set_axis_off();ax.set_title(title,fontsize=12,pad=0)
    return ax

fig=plt.figure(figsize=(13,8));panel(fig,(1,2,1),'standing','mechanical','Full mechanical envelope — unresolved clashes')
panel(fig,(1,2,2),'standing','costume','Qi Tian Da Sheng / Wukong costume occupancy')
fig.suptitle('Whole-body '+VER+' — same geometry and scale; NOT a build-ready robot',fontsize=14)
fig.text(.08,.045,'390 mm crown height candidate. Broad stance and actuator packaging remain engineering conflicts.',fontsize=10)
fig.savefig(OUT/'mechanical_and_costume.png',dpi=150,bbox_inches='tight');plt.close(fig)
fig=plt.figure(figsize=(16,7));panel(fig,(1,3,1),'standing','costume','Front',0,0)
panel(fig,(1,3,2),'standing','costume','Side',0,90);panel(fig,(1,3,3),'standing','costume','Rear',0,180)
fig.suptitle('Wukong concept occupancy — proportions from current mechanical candidate',fontsize=14)
fig.text(.08,.04,'Concept only: crown, monkey face, red/gold armor and boot interfaces. User visual approval pending.',fontsize=10)
fig.savefig(OUT/'qitian_views.png',dpi=150,bbox_inches='tight');plt.close(fig)
fig=plt.figure(figsize=(13,8));panel(fig,(1,2,1),'handover','costume','Bottle handover pose — collision audit pending')
panel(fig,(1,2,2),'getup_intermediate','mechanical','Get-up intermediate — task NOT validated')
fig.suptitle('Task pose candidates — not validated motions',fontsize=14)
fig.savefig(OUT/'task_poses.png',dpi=150,bbox_inches='tight');plt.close(fig)
print('Rendered mechanical/costume comparison, three views and task poses.')
