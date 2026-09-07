"""Scientific nominal vertical FOV comparison; no synthetic product appearance."""
import json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/camera_v27'
data=json.loads((ROOT/'08_electrical/camera_candidates_v27.json').read_text())
tasks=json.loads((ROOT/'03_system/camera_study_v27.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(12,8))
for ax,(name,pitch) in zip(axes.flat,[('D436',0),('D436',45),('D435i',0),('D435i',45)]):
    c=next(c for c in data['candidates'] if c['id']==name);a=math.radians(pitch);co,si=math.cos(a),math.sin(a)
    origin=[40+c['xyz_mm'][0]/2*co,tasks['body_origin_height_stand_mm']+135-c['xyz_mm'][0]/2*si]
    def point(z,v):return [origin[0]+co*z+si*v,origin[1]-si*z+co*v]
    for fov,near,color,label in [(c['depth_fov_deg'][1],195,'#39a2cf','depth, nominal Min-Z 195mm'),(c['rgb_fov_deg'][1],0,'#eaa747','RGB nominal angular FOV')]:
        t=math.tan(math.radians(fov/2));pts=[point(near,-near*t),point(800,-800*t),point(800,800*t),point(near,near*t)]
        ax.add_patch(Polygon(pts,facecolor=color,alpha=.18,edgecolor=color,label=label))
    ax.add_patch(Rectangle((148,0),64,220,facecolor='#91b391',alpha=.5,label='assumed bottle 64x220mm'))
    ax.plot(*origin,'ko');ax.axhline(0,color='k',linewidth=1)
    for key in ['bottle_grip_center','bottle_top_center','near_table_edge','opponent_near']:
        p=tasks['task_probe_points_mm'][key];ax.plot(p[0],p[2],'o',markersize=4);ax.annotate(key,(p[0],p[2]),xytext=(3,5),textcoords='offset points',fontsize=7)
    ax.set(xlim=(0,800),ylim=(-40,600),xlabel='forward X (mm)',ylabel='height above tabletop Z (mm)',title=f'{name}, assumed sensor pitch {pitch} deg')
    ax.grid(alpha=.2);ax.legend(fontsize=7,loc='upper right')
fig.suptitle('Nominal camera FOV vs tasks - not actual stereo coverage or validated neck motion')
fig.tight_layout(rect=(0,.07,1,.95));fig.text(.03,.025,'D436 848x480 Min-Z is inferred from D430. Optical offsets, stereo invalid band, real camera and complete bottle visibility still require verification.',fontsize=8)
fig.savefig(OUT/'fov_comparison.png',dpi=150);plt.close(fig)
