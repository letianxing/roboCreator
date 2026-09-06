import json,argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v08','v09','v11'],default='v08');args=parser.parse_args()
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad'/('ankle_'+args.variant)
data=json.loads((OUT/'render_mesh.json').read_text())
fig=plt.figure(figsize=(13,7),facecolor='white')
for index,az in enumerate((-60,130),1):
    ax=fig.add_subplot(1,2,index,projection='3d')
    for o in data:
        pts=np.asarray(o['points']);tri=pts[np.asarray(o['triangles'])]
        ax.add_collection3d(Poly3DCollection(tri,facecolor=o['color'],edgecolor=o['color'],linewidths=.03))
    ax.set(xlim=(-80,80),ylim=(-55,55),zlim=(0,135),xlabel='X / mm',ylabel='Y / mm',zlabel='Z / mm')
    ax.set_box_aspect((160,110,135));ax.view_init(elev=25,azim=az)
    ax.set_title('Front / side' if index==1 else 'Rear / bearing support')
fig.suptitle('Wukong ankle '+args.variant+' — solid mounting and bearing-support concept',fontsize=14)
fig.text(.07,.035,'Amber: moving yoke. Gold: removable boot-interface rims. Strength, fits and fasteners NOT released.',fontsize=10)
fig.savefig(OUT/'structural_preview.png',dpi=160,bbox_inches='tight');plt.close(fig)
print(OUT/'structural_preview.png')
