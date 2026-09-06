"""Scientific previews of current assembly and contact validation results."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v19'
names=['stand_candidate','cuff_knee_candidate','palm_transition_probe','alternating_handoff_midpoint']
titles=['Standing candidate','Cuff + knees','Palm + knees','Handoff midpoint: collision retained']
fig=plt.figure(figsize=(16,8))
for i,(name,title) in enumerate(zip(names,titles)):
    rows=json.loads((OUT/(name+'_mesh.json')).read_text());ax=fig.add_subplot(1,4,i+1,projection='3d');ax.set_proj_type('ortho')
    for r in rows:
        points=np.asarray(r['points']);triangles=points[np.asarray(r['triangles'])]
        color='#d6a12b' if r['role']=='shell' else '#db7726' if 'KneePad' in r['name'] or 'Gauntlet' in r['name'] else '#4383b7' if r['role']=='motor' else '#9a66a2' if r['role']=='electronics' else '#89929b'
        ax.add_collection3d(Poly3DCollection(triangles,facecolor=color,edgecolor='none',alpha=.75))
    ax.plot([-170,220,220,-170,-170],[-180,-180,180,180,-180],[0]*5,color='green',linewidth=.8)
    ax.set(xlim=(-200,240),ylim=(-235,235),zlim=(-5,430));ax.set_box_aspect((440,470,435),zoom=1.25);ax.view_init(elev=15,azim=-65);ax.set_axis_off();ax.set_title(title,fontsize=11)
fig.suptitle('v19 static/contact candidates — full recovery motion remains unvalidated',fontsize=13)
fig.text(.05,.065,'Orange: modeled contact pads. Blue: motor previews. Gold: provisional Wukong head/crown. Complete shoulder/elbow interfaces remain pending.',fontsize=9)
fig.subplots_adjust(top=.9,bottom=.12,wspace=.025);fig.savefig(OUT/'recovery_views.png',dpi=140);plt.close(fig)
balance=json.loads((ROOT/'05_dynamics/support_v19/balance_results.json').read_text());phases=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text());fig,axs=plt.subplots(1,2,figsize=(12,4.8))
rows=balance['direct_symmetric_handoff'];axs[0].plot([r['s'] for r in rows],[r['cuff_min_z_mm'] for r in rows],label='cuff pad');axs[0].plot([r['s'] for r in rows],[r['palm_min_z_mm'] for r in rows],label='palm');axs[0].axhline(0,color='black',ls='--');axs[0].set(title='Direct simultaneous handoff loses support',xlabel='interpolation fraction',ylabel='lowest pad corner / mm');axs[0].legend();axs[0].grid(alpha=.25)
rows=phases['toe_transfer_probe'];axs[1].plot([r['s'] for r in rows],[r['shoulder_to_fixed_hand_mm'] for r in rows],label='required reach');axs[1].axhline(rows[0]['absolute_arm_reach_bound_mm'],color='red',ls='--',label='absolute length bound');axs[1].set(title='Fixed-hand symmetric foot transfer fails reach',xlabel='interpolation fraction',ylabel='distance / mm');axs[1].legend();axs[1].grid(alpha=.25)
fig.suptitle('Rejected interpolation diagnostics: all samples retained');fig.tight_layout();fig.savefig(ROOT/'05_dynamics/support_v19/transition_diagnostics.png',dpi=140);plt.close(fig)
print('Saved recovery_views.png and transition_diagnostics.png')
