"""Scientific CAD overview, not a final Wukong appearance rendering."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'09_cad/uac_space_v24'
fig=plt.figure(figsize=(14,7))
for i,name in enumerate(['stand','lift_h114','lift_h120']):
    rows=json.loads((OUT/('routed_'+name+'_mesh.json')).read_text())
    ax=fig.add_subplot(1,3,i+1,projection='3d');ax.set_proj_type('ortho')
    for row in rows:
        n=row['name'];p=np.array(row['points']);f=p[np.array(row['triangles'])]
        color='#e13e3e' if n=='UAC_Bay_RESERVATION' else '#13b9d3' if n=='USB_Exit_RESERVATION' else '#eb9a19' if n.startswith('Array') else '#bf993e' if n in ('WukongHeadShell','CrownOccupancy') else '#437dac' if 'Motor' in n else '#92989d'
        alpha=.95 if ('RESERVATION' in n or n.startswith('ArrayRoute')) else .32
        ax.add_collection3d(Poly3DCollection(f,facecolor=color,edgecolor='none',alpha=alpha))
    ax.set(xlim=(-180,200),ylim=(-190,190),zlim=(-5,410));ax.set_box_aspect((380,380,415))
    ax.view_init(elev=17,azim=-55);ax.set_axis_off();ax.set_title(name)
fig.suptitle('v24 available-space study | 4 sampled poses, NOT hardware fit approval')
fig.text(.04,.10,'RED: board/mount reservation. CYAN: USB plug/exit reservation. ORANGE: array exit and cable corridor.',fontsize=10)
fig.text(.04,.065,'PCB size/holes/mass and actual cables unknown. Corridor has square bends: bend radius and electrical length NOT validated.',fontsize=9)
fig.text(.04,.03,'Existing Wukong head/crown is reference geometry, NOT the final appearance. No manufacturing, full recovery or CAE release.',fontsize=9)
fig.subplots_adjust(top=.90,bottom=.14,wspace=0)
fig.savefig(OUT/'space_review.png',dpi=150);plt.close(fig)
