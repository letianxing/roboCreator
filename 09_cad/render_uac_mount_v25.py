"""Native cassette detail + robot assembly, scientific visualization only."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/uac_mount_v25_service'
fig=plt.figure(figsize=(14,7))
for i,name in enumerate(['mount_native','stand','lift_h114']):
    ax=fig.add_subplot(1,3,i+1,projection='3d');ax.set_proj_type('ortho')
    rows=json.loads((OUT/(name+'_mesh.json')).read_text())
    for r in rows:
        n=r['name'];p=np.array(r['points']);f=p[np.array(r['triangles'])]
        col='#df922d' if n=='ElectronicsCarrier' else '#c93150' if n=='UACCassette' else '#10984c' if 'Cable' in n else '#523692' if ('Liner' in n or 'Tie' in n) else '#3c9bb6' if 'Space' in n else '#657d9a'
        alpha=.17 if 'Space' in n else .94 if (n in ['ElectronicsCarrier','UACCassette'] or n.startswith(('Array','UAC','Insulation'))) else .27
        ax.add_collection3d(Poly3DCollection(f,facecolor=col,edgecolor='none',alpha=alpha))
    if name=='mount_native':
        ax.set(xlim=(-85,65),ylim=(-70,70),zlim=(35,125));ax.set_box_aspect((150,140,90));ax.view_init(elev=24,azim=-50)
    else:
        ax.set(xlim=(-190,230),ylim=(-190,190),zlim=(-5,410));ax.set_box_aspect((420,380,415));ax.view_init(elev=18,azim=-55)
    ax.set_axis_off();ax.set_title(name)
fig.suptitle('v25 removable cassette + soft-lined tie anchor | RESEARCH CAD, NOT manufacturing release')
fig.text(.03,.08,'Orange: revised carrier. Red: removable cassette. Purple: split liner / provisional tie. Green: R6 / diameter4 cable proxy.',fontsize=9)
fig.text(.03,.04,'Board dimensions/holes, real cable, tightening, insulation, impact and CAE remain unverified. Final Wukong appearance pending.',fontsize=9)
fig.subplots_adjust(top=.89,bottom=.13,wspace=0);fig.savefig(OUT/'mount_review.png',dpi=150);plt.close(fig)
