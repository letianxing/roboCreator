"""Scientific previews of the actual exported leg and gauntlet poses."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'09_cad/single_leg_v17/interface_revision'
data=json.loads((folder/'render_mesh.json').read_text());result=json.loads((folder/'integration_results.json').read_text())
fig=plt.figure(figsize=(16,7))
for i,pose in enumerate(['stand','squat','deep_squat_probe','lift_probe']):
    ax=fig.add_subplot(1,4,i+1,projection='3d');ax.set_proj_type('ortho')
    for obj in (x for x in data if x['pose']==pose):
        pts=np.asarray(obj['points']);tris=pts[np.asarray(obj['triangles'])]
        color='#3878ae' if obj['role']=='motor' else '#dbb04e' if 'Roll' in obj['name'] and 'Motor' not in obj['name'] else '#c73535' if obj['role']=='keepout' else '#8a939d'
        ax.add_collection3d(Poly3DCollection(tris,facecolor=color,edgecolor='none',alpha=.8))
    row=next(x for x in result['records'] if x['pose']==pose);xyz=np.array([x['origin_mm'] for x in row['joints']]);ax.plot(xyz[:,0],xyz[:,1],xyz[:,2],'-o',color='black',markersize=3)
    ax.set(xlim=(-140,140),ylim=(-110,110),zlim=(0,330));ax.set_box_aspect((280,220,330));ax.view_init(elev=12,azim=-55);ax.set_axis_off();ax.set_title(pose.replace('_',' '),fontsize=11)
fig.suptitle('v17 six-joint leg — revised bearing pockets / shafts; blue motor references',fontsize=13)
fig.text(.08,.065,'Isolated configurations with matching scale. Manufacturing interfaces, costume, complete roll sweep and wholebody balance remain pending.',fontsize=9)
fig.subplots_adjust(top=.91,bottom=.10,wspace=.01);fig.savefig(folder/'leg_views.png',dpi=130);plt.close(fig)

hand=ROOT/'09_cad/gauntlet_v17';meshes=json.loads((hand/'render_mesh.json').read_text());replay=json.loads((hand/'replay_results.json').read_text())
fig=plt.figure(figsize=(13,6))
for i,row in enumerate(replay['records']):
    ax=fig.add_subplot(1,3,i+1,projection='3d');ax.set_proj_type('ortho')
    for obj in (x for x in meshes if x['pose']==row['name']):
        points=np.asarray(obj['points']);triangles=points[np.asarray(obj['triangles'])];color='#d5a82d' if obj['name']=='GauntletPrimaryPad' else '#3779b4' if 'Motor' in obj['name'] else '#999999'
        ax.add_collection3d(Poly3DCollection(triangles,facecolor=color,edgecolor='none',alpha=.85))
    ax.plot([40,240,240,40,40],[40,40,190,190,40],[0]*5,color='green')
    ax.set(xlim=(30,260),ylim=(35,190),zlim=(-25,160));ax.set_box_aspect((230,155,185));ax.view_init(elev=15,azim=-75);ax.set_axis_off()
    tag='pad/ground candidate' if row['IK']['finite_search_pose_found'] and not row['has_ground_penetration_over_0_5mm'] else 'rejected pose'
    ax.set_title(str(row['IK']['architecture_dof'])+' DOF / '+tag,fontsize=10)
fig.suptitle('v17 forearm-fixed contact: solved position + pad orientation + actual hand geometry')
fig.text(.08,.06,'Green: ground. No torso or leg support feasibility is implied. 180-degree gauntlet mounting is a new candidate.',fontsize=9)
fig.subplots_adjust(top=.87,bottom=.13,wspace=.04);fig.savefig(hand/'contact_views.png',dpi=130);plt.close(fig)
print('Saved leg_views.png and contact_views.png')
