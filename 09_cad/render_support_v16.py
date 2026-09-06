"""Render saved v16 local mechanism meshes and load-path curves."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];C=ROOT/'09_cad/support_mechanisms_v16';D=ROOT/'05_dynamics'
mesh=json.loads((C/'render_mesh.json').read_text());colors={'load_structure':'#9ba3aa','drive_structure':'#d69a2d','motor_envelope':'#347cc1','contact_pad':'#222222','reference':'#8064a2'}
poses=['ankle_m25_p0','ankle_p0_p0','ankle_p25_p0','hand_p0_p0','hand_m40_p0','hand_m80_p0']
fig=plt.figure(figsize=(14,8))
for i,pose in enumerate(poses):
    ax=fig.add_subplot(2,3,i+1,projection='3d');ax.set_proj_type('ortho')
    for o in (x for x in mesh if x['pose']==pose):
        pts=np.asarray(o['points']);tris=pts[np.asarray(o['triangles'])]
        ax.add_collection3d(Poly3DCollection(tris,facecolor=colors[o['category']],edgecolor='#333333',linewidths=.15,alpha=.82))
    if pose.startswith('ankle'):ax.set(xlim=(-65,65),ylim=(-55,55),zlim=(-10,140))
    else:ax.set(xlim=(-90,60),ylim=(-50,50),zlim=(-15,100))
    ax.set_box_aspect((1,1,1));ax.view_init(elev=19,azim=-48);ax.set_axis_off();ax.set_title(pose.replace('_',' '),fontsize=10)
fig.suptitle('v16 local load-path concepts — NOT manufacturing or whole-body validation',fontsize=13)
fig.text(.08,.035,'Blue: motor envelope  |  grey: aluminum reference structure  |  gold: roll linkage  |  black: contact pad',fontsize=9)
fig.subplots_adjust(top=.90,bottom=.08,wspace=.05,hspace=.12);fig.savefig(C/'mechanism_views.png',dpi=140);plt.close(fig)

x=json.loads((D/'support_load_paths_v16.json').read_text());fig,axs=plt.subplots(1,2,figsize=(12,4.5))
for tau in (0.5,1.3,2.0,4.1):
    rows=[r for r in x['ankle_linkage']['rows'] if r['torque_Nm']==tau]
    axs[0].plot([r['roll_deg'] for r in rows],[r['rod_axial_force_N'] for r in rows],label=f'{tau:g} Nm')
axs[0].set(title='Ideal parallel-crank rod force',xlabel='ankle roll / deg',ylabel='rod axial force / N');axs[0].grid(alpha=.25);axs[0].legend()
rows=[r for r in x['gauntlet_contact']['rows'] if r['robot_mass_kg']==4 and r['pad_stiffness_each_N_mm']==50]
axs[1].bar([str(r['load_multiplier'])+'x' for r in rows],[r['gauntlet_force_N'] for r in rows],label='gauntlet bypass')
axs[1].bar([str(r['load_multiplier'])+'x' for r in rows],[r['palm_force_N'] for r in rows],bottom=[r['gauntlet_force_N'] for r in rows],label='palm / wrist')
axs[1].set(title='Assumed contact sharing: 4 kg, 50 N/mm pads',xlabel='load multiplier',ylabel='per-arm vertical force / N');axs[1].grid(axis='y',alpha=.25);axs[1].legend()
fig.suptitle('v16 sensitivity only — real pad stiffness, tolerances and impact are unmeasured');fig.tight_layout();fig.savefig(D/'support_load_paths_v16.png',dpi=140);plt.close(fig)
print('Rendered mechanism_views.png and support_load_paths_v16.png')
