"""Offline technical visualization of FreeCAD triangles, not a design validation."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'09_cad/ankle_v07_axial'
data=json.loads((OUT/'render_mesh.json').read_text())
fig=plt.figure(figsize=(10,8),facecolor='white')
ax=fig.add_subplot(111,projection='3d',computed_zorder=False)
for o in data:
    pts=np.asarray(o['points']);tris=pts[np.asarray(o['triangles'])]
    coll=Poly3DCollection(tris,facecolor=o['color'],linewidths=0.05,edgecolor=o['color'],alpha=1)
    ax.add_collection3d(coll)
ax.plot([-65,80],[0,0],[38,38],color='#d54343',linewidth=2,label='Roll axis X, z=38 mm')
ax.plot([30,30],[-40,45],[38,38],color='#28985d',linewidth=2,label='Pitch axis Y, z=38 mm')
ax.plot([-6.9,0,0,30],[0,0,-24,-24],[38,38,38,38],color='#ee941e',linewidth=2,
        label='Yoke connection sketch (not a bracket)')
ax.set(xlim=(-75,90),ylim=(-65,65),zlim=(0,165),xlabel='X / mm',ylabel='Y / mm',zlabel='Z / mm')
ax.set_box_aspect((165,130,165));ax.view_init(elev=24,azim=-58)
ax.set_title('Wukong ankle v0.7 — axial-offset packaging draft',pad=20)
ax.legend(loc='upper left',fontsize=8)
fig.text(.1,.025,'Vendor surface models + foot/shank proxies. Mounts, bearings and load verification pending.',fontsize=9)
fig.savefig(OUT/'preview_verified.png',dpi=160,bbox_inches='tight')
plt.close(fig)
print(OUT/'preview_verified.png')
