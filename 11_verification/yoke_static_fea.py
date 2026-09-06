"""Linear C3D4 screening of the v08 yoke with mesh refinement.
N, mm, MPa. Boundary idealizations do not certify the real bolted/bearing assembly.
"""
import json,math,subprocess,os,re,hashlib,argparse,shutil
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v08','v09'],default='v08')
parser.add_argument('--sizes',type=float,nargs='+',default=[2.5,1.8,1.2])
parser.add_argument('--bearing',choices=['radial','bonded_transverse'],default='radial')
args=parser.parse_args()
ROOT=Path(__file__).resolve().parents[1]
OLDOUT=ROOT/'11_verification'/('yoke_fea_v09' if args.variant=='v08' else 'yoke_fea_reinforced_v09')
OUT=Path(str(OLDOUT)+'_radial') if args.bearing=='radial' else OLDOUT;OUT.mkdir(exist_ok=True)
STEP=ROOT/'09_cad'/('ankle_'+args.variant)/'RollTransferYoke.step'
GMSH=r'G:\freeCAD\bin\gmsh.exe';CCX=r'G:\freeCAD\bin\ccx.exe'
def run(cmd,cwd,log):
    env=os.environ.copy();env.update(OMP_NUM_THREADS='2',CCX_NPROC_RESULTS='2')
    r=subprocess.run(cmd,cwd=cwd,env=env,capture_output=True,text=True,errors='replace',
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),timeout=240)
    log.write_text(r.stdout+'\n'+r.stderr,encoding='utf-8')
    if r.returncode:raise RuntimeError(f'Failed {cmd[0]}: {r.returncode}; {log}')

def mesh_read(p):
    lines=p.read_text().splitlines();i=lines.index('$Nodes');n=int(lines[i+1])
    nodes={int(t[0]):np.array(list(map(float,t[1:4]))) for t in (line.split() for line in lines[i+2:i+2+n])}
    i=lines.index('$Elements');n=int(lines[i+1]);tet=[];tri=[]
    for line in lines[i+2:i+2+n]:
        t=list(map(int,line.split()));etype=t[1];ids=t[3+t[2]:]
        if etype==4:tet.append(ids)
        if etype==2:tri.append(ids)
    assert tet and tri
    return nodes,tet,tri

def write_deck(folder,nodes,tets,tris):
    fixed=[n for n,(x,y,z) in nodes.items() if abs(x+9.9)<1e-5 and y*y+(z-45)**2<18.5**2+1e-4]
    radial=[n for n,(x,y,z) in nodes.items() if -3.9001<=x<=4.1001 and abs(math.hypot(y,z-45)-4.99)<1e-4]
    areas={};area=0
    for ids in tris:
        pts=np.array([nodes[n] for n in ids])
        if np.max(np.abs(pts[:,1]-19.1))>1e-5:continue
        # Only annular pitch stator face, not any unrelated coplanar patch.
        if np.min(pts[:,0])<6.999 or np.max(pts[:,0])>71.001:continue
        a=np.linalg.norm(np.cross(pts[1]-pts[0],pts[2]-pts[0]))/2
        area+=a
        for n in ids:areas[n]=areas.get(n,0)+a/3
    assert len(fixed)>10 and len(radial)>5 and len(areas)>10 and area>1000,(len(fixed),len(radial),len(areas),area)
    deck=['*HEADING','V08 YOKE SCREENING; unit N mm; Fz=-100N, not impact requirement','*NODE,NSET=ALL']
    deck += [f'{n},'+','.join(f'{v:.12g}' for v in p) for n,p in nodes.items()]
    deck+=['*ELEMENT,TYPE=C3D4,ELSET=ALL_ELEMENTS']
    deck += [f'{i},'+','.join(map(str,t)) for i,t in enumerate(tets,1)]
    for name,ids in [('FIXED',fixed),('RADIAL',radial)]:
        deck.append('*NSET,NSET='+name)
        deck += [','.join(map(str,ids[j:j+16])) for j in range(0,len(ids),16)]
    if args.bearing=='radial':deck+=['*TRANSFORM,NSET=RADIAL,TYPE=C','0,0,45,1,0,45']
    deck += ['*MATERIAL,NAME=AL_ASSUMED','*ELASTIC','69000,0.33','*SOLID SECTION,ELSET=ALL_ELEMENTS,MATERIAL=AL_ASSUMED',
             '*BOUNDARY','FIXED,1,3,0',('RADIAL,1,1,0' if args.bearing=='radial' else 'RADIAL,2,3,0'),'*STEP','*STATIC','*CLOAD']
    deck += [f'{n},3,{-100*a/area:.12g}' for n,a in areas.items()]
    deck += ['*NODE PRINT,NSET=ALL,GLOBAL=YES','U','RF','*EL PRINT,ELSET=ALL_ELEMENTS','S',
             '*NODE FILE','U','*EL FILE','S','*END STEP']
    (folder/'yoke.inp').write_text('\n'.join(deck)+'\n')
    manifest={'fixed_nodes':fixed,'radial_nodes':radial,'load_area_mm2':area,
              'load_nodes':{str(n):float(-100*a/area) for n,a in areas.items()},
              'force_N':[0,0,-100],'E_MPa':69000,'nu':.33}
    (folder/'boundary_conditions.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

def read_dat(path):
    state=None;disp={};forces={};stress=[]
    for line in path.read_text(errors='replace').splitlines():
        low=line.strip().lower()
        if low.startswith('displacements'):state='u';continue
        if low.startswith('forces'):state='rf';continue
        if low.startswith('stresses'):state='s';continue
        fields=line.split()
        if not fields or not fields[0].isdigit():continue
        if state in ('u','rf') and len(fields)==4:
            v=list(map(float,fields[1:]));(disp if state=='u' else forces)[int(fields[0])]=v
        if state=='s' and len(fields)==8:
            v=list(map(float,fields[2:]));sx,sy,sz,txy,txz,tyz=v
            vm=math.sqrt(.5*((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)+3*(txy*txy+txz*txz+tyz*tyz))
            stress.append((int(fields[0]),vm,v))
    assert disp and forces and stress,(len(disp),len(forces),len(stress))
    return disp,forces,stress

def main():
    summary=[]
    for h in args.sizes:
        folder=OUT/f'mesh_{h}';folder.mkdir(exist_ok=True)
        geo=f'''SetFactory("OpenCASCADE");
v()=ShapeFromFile("{STEP.as_posix()}");
Mesh.MeshSizeMin={h}; Mesh.MeshSizeMax={h};
Mesh.ElementOrder=1; Mesh.MshFileVersion=2.2; Mesh.SaveAll=1;
'''
        (folder/'mesh.geo').write_text(geo)
        print('Meshing',h,flush=True)
        oldmesh=OLDOUT/f'mesh_{h}'/'mesh.msh'
        if args.bearing=='radial' and oldmesh.exists():
            shutil.copy2(oldmesh,folder/'mesh.msh')
            (folder/'gmsh.log').write_text('Reused same-geometry mesh from '+str(oldmesh)+'\n')
        else:run([GMSH,'mesh.geo','-3','-o','mesh.msh','-nt','2'],folder,folder/'gmsh.log')
        nodes,tets,tris=mesh_read(folder/'mesh.msh');bc=write_deck(folder,nodes,tets,tris)
        print('Solving',h,len(nodes),'nodes',len(tets),'tetrahedra',flush=True)
        run([CCX,'-i','yoke'],folder,folder/'solver.log')
        u,rf,stress=read_dat(folder/'yoke.dat')
        # RF with transformed SPC/MPC is incomplete in CalculiX. Recover Fint from C3D4 stress.
        ids=np.array(tets,dtype=int);pts=np.array([[nodes[n] for n in t] for t in tets])
        matrices=np.concatenate([np.ones((len(tets),4,1)),pts],axis=2)
        grads=np.linalg.inv(matrices)[:,1:,:]
        volumes=np.abs(np.linalg.det(matrices))/6
        sv=np.array([x[2] for x in sorted(stress)])
        tensors=np.zeros((len(tets),3,3))
        for a,b,i in [(0,0,0),(1,1,1),(2,2,2),(0,1,3),(0,2,4),(1,2,5)]:
            tensors[:,a,b]=sv[:,i];tensors[:,b,a]=sv[:,i]
        f_element=np.einsum('eij,ejn,e->eni',tensors,grads,volumes)
        f_internal=np.zeros((max(nodes)+1,3));np.add.at(f_internal,ids.ravel(),f_element.reshape(-1,3))
        fixed_force=f_internal[bc['fixed_nodes']].sum(axis=0)
        radial_force=f_internal[bc['radial_nodes']].sum(axis=0)
        react=fixed_force+radial_force
        support_moment=sum((np.cross(nodes[n],f_internal[n]) for n in bc['fixed_nodes']+bc['radial_nodes']),np.zeros(3))
        load_moment=sum((np.cross(nodes[int(n)],np.array([0,0,f])) for n,f in bc['load_nodes'].items()),np.zeros(3))
        energy_internal=.5*sum(np.dot(u[n],f_internal[n]) for n in nodes)
        energy_external=.5*sum(f*u[int(n)][2] for n,f in bc['load_nodes'].items())
        loaded_u=sum(-f*u[int(n)][2]/100 for n,f in bc['load_nodes'].items())
        max_node=max(u,key=lambda n:np.linalg.norm(u[n]))
        stressvals=np.array([s[1] for s in stress])
        row={'mesh_size_mm':h,'node_count':len(nodes),'element_count':len(tets),
             'max_displacement_mm':float(np.linalg.norm(u[max_node])),
             'max_displacement_node':max_node,'load_weighted_uz_mm':loaded_u,
             'peak_von_mises_MPa':float(stressvals.max()),'p95_von_mises_MPa':float(np.percentile(stressvals,95)),
             'reaction_sum_N':react.tolist(),'bearing_resultant_N':radial_force.tolist(),
             'force_balance_residual_N':float(np.linalg.norm(react+[0,0,-100])),
             'moment_balance_residual_Nmm':float(np.linalg.norm(support_moment+load_moment)),
             'energy_relative_error':abs(energy_internal-energy_external)/abs(energy_external),
             'bearing_constraint':args.bearing}
        (folder/'result_summary.json').write_text(json.dumps(row,indent=2)+'\n')
        summary.append(row);print(json.dumps(row),flush=True)
    out={'status':'linear_elastic_local_screen_only','bearing_constraint':args.bearing,'geometry_sha256':hashlib.sha256(STEP.read_bytes()).hexdigest(),
         'results':summary,'assumptions':['C3D4 linear tetrahedra, E69GPa/nu0.33 assumed aluminum, no certified material',
          'Roll output face fully clamped; radial mode only restricts radial displacement, tangential/axial free; bearing compliance omitted',
          '100N downward on pitch stator annular interface by triangle-area nodal weights, not a measured landing load',
          'No inertia, selfweight, fatigue, bolt pretension or plasticity; sharp re-entrant corners may create stress singularities',
          'No allowable-load or whole-robot safety certification']}
    summary_path=OUT/('summary.json' if args.sizes==[2.5,1.8,1.2] else 'summary_custom.json')
    summary_path.write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':main()
