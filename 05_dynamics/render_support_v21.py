"""Scientific support-margin plot; failed geometry remains visible."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'05_dynamics/support_v21'
r=json.loads((ROOT/'09_cad/body_v21/single_palm_extended.json').read_text())['selected'][0]
rows=json.loads((OUT/'contact_margins.json').read_text())['cases'];p=np.array(r['support_points_mm'])[:,:2];h=ConvexHull(p);verts=np.r_[h.vertices,h.vertices[0]];com=r['COM_mm']
fig,ax=plt.subplots(1,2,figsize=(12,5))
ax[0].fill(p[verts,0],p[verts,1],alpha=.12,color='teal');ax[0].plot(p[verts,0],p[verts,1],color='teal',label='support polygon')
ax[0].scatter(p[:4,0],p[:4,1],label='left sole points');ax[0].scatter(*p[4],marker='s',label='right knee');ax[0].scatter(*p[5],marker='^',label='right palm');ax[0].scatter(com[0],com[1],marker='x',s=90,color='red',label='COM projection')
ax[0].set(title='120 mm body height: only 1.37 mm margin',xlabel='world X / mm',ylabel='world Y / mm');ax[0].axis('equal');ax[0].legend(fontsize=8);ax[0].grid(alpha=.25)
local=[r for r in rows if r['scene']=='local_lift'];local.sort(key=lambda r:r['h_mm'])
ax[1].plot([r['h_mm'] for r in local],[r['support_polygon_margin_mm'] for r in local],color='gray',lw=1)
for good,label,color in [(True,'geometry + vertical balance pass','teal'),(False,'geometry or balance failed','crimson')]:
    selected=[r for r in local if r['geometry_and_static_pass']==good]
    ax[1].scatter([r['h_mm'] for r in selected],[r['support_polygon_margin_mm'] for r in selected],label=label,color=color)
ax[1].axhline(0,color='black',ls='--',lw=1);ax[1].set(title='Local lift probe: margin is not the only gate',xlabel='body reference height / mm',ylabel='signed support margin / mm');ax[1].grid(alpha=.25);ax[1].legend(fontsize=8)
fig.suptitle('v21 quasi-static diagnostics — not a dynamic or hardware safety validation');fig.tight_layout();fig.savefig(OUT/'support_review.png',dpi=150);plt.close(fig)
