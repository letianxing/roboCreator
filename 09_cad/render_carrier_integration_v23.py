"""Scientific assembly preview: preserve distinction between reference blocks and arrived hardware."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v23_fit'
fig=plt.figure(figsize=(14,6))
for i,name in enumerate(['stand','lift_h114','lift_h120']):
    rows=json.loads((OUT/(name+'_mesh.json')).read_text());ax=fig.add_subplot(1,3,i+1,projection='3d');ax.set_proj_type('ortho')
    for r in rows:
        pts=np.array(r['points']);faces=pts[np.array(r['triangles'])]
        color='#ca9231' if r['role']=='shell' else '#d56e30' if r['name']=='ElectronicsCarrier' else '#6b417a' if r['role']=='retainer' else '#4c9b5b' if r['role']=='keepout' else '#4384ad' if r['role']=='motor' else '#a27ab5' if r['role']=='electronics' else '#90999f'
        ax.add_collection3d(Poly3DCollection(faces,facecolor=color,edgecolor='none',alpha=.5 if r['role']=='keepout' else .85))
    ax.set(xlim=(-210,220),ylim=(-210,210),zlim=(-5,410));ax.set_box_aspect((430,420,415));ax.view_init(elev=18,azim=-65);ax.set_axis_off();ax.set_title(name)
fig.suptitle('v23 reference carrier integrated — sampled geometry, not manufacturing release')
fig.text(.045,.05,'Orange: carrier. Purple: retainers. Green: cable/connector space probe. Actual arrived UAC board is NOT included; microphone dimensions remain provisional.',fontsize=9)
fig.subplots_adjust(top=.9,bottom=.12,wspace=0);fig.savefig(OUT/'integration_review.png',dpi=150);plt.close(fig)
