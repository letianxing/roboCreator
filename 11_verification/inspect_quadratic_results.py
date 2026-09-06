"""Locate stress peaks and check interpolation patch identities."""
import json
from pathlib import Path
import numpy as np
import quadratic_yoke_fea as q
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'11_verification/yoke_c3d10_v10'
# Partition, derivative and affine patch checks, including curved midside geometry.
corners=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1.]])
xyz=np.vstack([corners,[(corners[a]+corners[b])/2 for a,b in q.EDGES]])
xyz[4]+=[0,.03,.02]
affine=np.array([[.01,.02,.03],[-.02,.01,.02],[.03,0,.01]])
ue=xyz@affine.T+[1,2,3]
for l in [np.array([.25]*4),np.array([.4,.1,.2,.3])]:
    n,g=q.tet_shape(l);assert abs(n.sum()-1)<1e-12;assert np.linalg.norm(g.sum(axis=0))<1e-12
    jac=xyz.T@g;gx=g@np.linalg.inv(jac)
    assert np.allclose(ue.T@gx,affine,atol=1e-12)
rows=[]
for folder in sorted(OUT.glob('mesh_*')):
    if not (folder/'summary.json').exists():continue
    r=json.loads((folder/'summary.json').read_text());nodes,tets,tris=q.read_mesh(folder/'mesh.msh')
    peak=tets[r['peak_element']-1];corner=np.array([nodes[int(n)] for n in peak[:4]])
    r['peak_element_corner_centroid_mm']=corner.mean(axis=0).tolist()
    assert r['min_gauss_jacobian']>0
    assert r['force_residual_N']<.002 and r['moment_residual_Nmm']<.1
    assert r['energy_relative_error']<1e-4
    rows.append(r)
rows.sort(key=lambda r:-r['h_mm'])
result={'patch_test':'partition/derivatives/curved affine displacement passed','meshes':rows}
(OUT/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
for r in rows:print(r['h_mm'],'max_u',r['max_u_mm'],'peak_vm',r['peak_vm_MPa'],'peak_centroid',r['peak_element_corner_centroid_mm'])
