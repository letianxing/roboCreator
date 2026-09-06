"""Equivalent grounded connector stiffness study. Not pretension/contact/thread analysis."""
import json,math
from pathlib import Path
import numpy as np
import quadratic_yoke_fea as q
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'11_verification/bolt_compliance_v11';OUT.mkdir(exist_ok=True)
MESH=ROOT/'11_verification/yoke_c3d10_v10/mesh_1.8/mesh.msh'
q.FIXTURE='bolt_bores'
nodes,tets,tris=q.read_mesh(MESH);bc=q.boundary(nodes,tris)
fix,radial,forces,area=bc
holes=[(11.69134295109,51.75),(-11.69134295109,51.75),(0,31.5)]
groups=[[n for n in fix if abs(math.hypot(nodes[n][1]-y,nodes[n][2]-z)-1.4)<1e-4] for y,z in holes]
assert sum(map(len,groups))==len(fix)
# Nominal thread tensile-area approximation. All stiffness values are assumptions.
diameter=2.5;pitch=.45;At=math.pi/4*(diameter-.9382*pitch)**2
E=210000.;nu=.3;length=6.;G=E/(2*(1+nu))
kref=np.array([E*At/length,G*At/length,G*At/length])
rows=[]
for scale in (.05,.2,1.):
    folder=OUT/f'factor_{scale}';folder.mkdir(exist_ok=True)
    q.write_deck(folder,nodes,tets,bc)
    text=(folder/'yoke.inp').read_text();assert 'FIX,1,3,0\n' in text
    text=text.replace('FIX,1,3,0\n','')
    cards=[];springs={};eid=len(tets)+1
    for b,ids in enumerate(groups):
        k=kref*scale/len(ids)
        for d in range(3):
            name=f'B{b}_D{d}'
            cards.append('*ELEMENT,TYPE=SPRING1,ELSET='+name)
            for n in ids:cards.append(f'{eid},{n}');eid+=1
            cards+=['*SPRING,ELSET='+name,str(d+1),f'{k[d]:.10f}']
        for n in ids:springs[n]=k.copy()
    text=text.replace('*MATERIAL,NAME=AL','\n'.join(cards)+'\n*MATERIAL,NAME=AL',1)
    (folder/'yoke.inp').write_text(text)
    print('Solving equivalent connector factor',scale,flush=True)
    q.run([r'G:\freeCAD\bin\ccx.exe','-i','yoke'],folder,folder/'solver.log')
    u,stress=q.read_solution(folder/'yoke.dat');fint,minjac,volume=q.integrate_internal(nodes,tets,u)
    bolt_forces=[sum((-springs[n]*u[n] for n in ids),np.zeros(3)) for ids in groups]
    bearing=fint[radial].sum(axis=0);total=sum(bolt_forces,np.zeros(3))+bearing
    mismatch=max(float(np.linalg.norm(fint[n]+springs[n]*u[n])) for n in fix)
    moment=sum((np.cross(nodes[n],-springs[n]*u[n]) for n in fix),np.zeros(3))
    moment+=sum((np.cross(nodes[n],fint[n]) for n in radial),np.zeros(3))
    moment+=sum((np.cross(nodes[n],[0,0,f]) for n,f in forces.items()),np.zeros(3))
    ein=.5*sum(u[n]@fint[n] for n in nodes)
    espring=.5*sum(np.dot(springs[n]*u[n],u[n]) for n in fix)
    eout=.5*sum(f*u[n][2] for n,f in forces.items())
    row={'factor':scale,'equivalent_total_stiffness_per_bolt_N_per_mm':(kref*scale).tolist(),
         'max_u_mm':max(float(np.linalg.norm(v)) for v in u.values()),
         'load_weighted_uz_mm':sum(-f*u[n][2]/100 for n,f in forces.items()),
         'peak_vm_MPa':max(x[2] for x in stress),'bolt_reaction_vectors_N':[f.tolist() for f in bolt_forces],
         'bearing_reaction_N':bearing.tolist(),'force_residual_N':float(np.linalg.norm(total+[0,0,-100])),
         'moment_residual_Nmm':float(np.linalg.norm(moment)),
         'connector_equilibrium_max_error_N':mismatch,
         'energy_relative_error':abs(ein+espring-eout)/abs(eout)}
    (folder/'summary.json').write_text(json.dumps(row,indent=2)+'\n');rows.append(row);print(json.dumps(row),flush=True)
result={'status':'equivalent_connector_sensitivity_only','source_mesh':str(MESH),'mesh_size_mm':1.8,
    'reference':{'E_MPa':E,'nu':nu,'nominal_d_mm':diameter,'pitch_mm':pitch,'effective_length_mm':length,'area_mm2':At},
    'results':rows,'limits':['No measured connector stiffness, preload, contact or friction',
        'Spring constants include hypothetical reduction factors; not real material identification',
        'Independent grounded nodal springs distributed equally within each bore: not rigid bolt heads',
        'Bilateral linear springs do not model opening, clearance, slip or thread failure',
        'Same geometry and mesh only; local stress not convergence-certified']}
(OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
