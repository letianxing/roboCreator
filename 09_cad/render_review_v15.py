"""Static scientific figures of generated shapes and computed time histories."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/modules_v15'
mesh=json.loads((OUT/'render_mesh.json').read_text());s=json.loads((OUT/'packaging_results.json').read_text())
fig=plt.figure(figsize=(12,10))
for i,r in enumerate(s['modules']):
    ax=fig.add_subplot(2,2,i+1,projection='3d');ax.set_proj_type('ortho');zbase=40 if i<2 else 0
    for o in (o for o in mesh if o['module']==r['module']):
        pts=np.asarray(o['points']);tris=pts[np.asarray(o['triangles'])]
        ax.add_collection3d(Poly3DCollection(tris,facecolor=o['color'],edgecolor='#333333',linewidths=.25,alpha=.70))
    if i<2:
        ax.plot([-55,55,55,-55,-55],[-32.5,-32.5,32.5,32.5,-32.5],[0]*5,color='green',linewidth=2)
    ax.set(xlim=(-85,55),ylim=(-55,85),zlim=(zbase-60,zbase+80),xlabel='X mm',ylabel='Y mm',zlabel='Z mm')
    ax.set_box_aspect((1,1,1));ax.view_init(elev=20,azim=-50)
    size=r['selected']['swept_size_mm'];ax.set_title(r['module']+' / swept '+ ' x '.join(f'{v:.0f}' for v in size)+' mm',fontsize=10)
fig.suptitle('v15 MOTOR ENVELOPES ONLY — frames, output links and Wukong costume absent',fontsize=12)
fig.text(.1,.035,'Same axis scale. Green: 110 x 65 mm sole reference. Neutral geometry shown; size labels include sampled roll motion.',fontsize=9)
fig.subplots_adjust(hspace=.15,wspace=.15,top=.90,bottom=.09);fig.savefig(OUT/'module_comparison.png',dpi=130);plt.close(fig)
folder=ROOT/'05_dynamics/trajectory_v15';fig,axs=plt.subplots(2,2,figsize=(12,8))
for name in ['leg_A_0.5s','leg_B_0.25s','leg_B_wrist_upgrade_0.25s']:
    r=json.loads((folder/(name+'.json')).read_text());rows=r['rows'];t=[x['time_s'] for x in rows]
    axs[0,0].plot(t,[x['tau_Nm'][1] for x in rows],label=name)
    axs[0,1].plot(t,[abs(x['dq_rad_s'][1])*60/(2*np.pi) for x in rows],label=name)
    axs[1,0].plot(t,[x['required_CoP_x_m']*1000 for x in rows],label=name)
r=json.loads((folder/'arm_lift_0.6s.json').read_text())
for j,label in enumerate(['shoulder pitch','elbow','wrist pitch']):axs[1,1].plot([x['time_s'] for x in r['rows']],[x['tau_Nm'][j] for x in r['rows']],label=label)
axs[0,0].axhline(1.3,color='black',ls='--',label='AK rated magnitude reference')
axs[0,0].axhline(-1.3,color='black',ls='--')
axs[0,1].axhline(46,color='black',ls='--',label='XM no-load at 12V')
titles=['Knee signed torque / Nm','Knee speed magnitude / rpm','Required planar CoP / mm','Arm lift proxy torques / Nm']
for ax,title in zip(axs.flat,titles):ax.set_title(title);ax.set_xlabel('time / s');ax.grid(alpha=.25);ax.legend(fontsize=7)
fig.suptitle('v15 conditional planar time histories — NOT takeoff or wholebody task validation')
fig.tight_layout();fig.savefig(folder/'load_curves.png',dpi=130);plt.close(fig)
print('Saved module_comparison.png and load_curves.png')
