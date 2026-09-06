"""C3D10 comparison, consistent quadratic face loading and independent equilibrium recovery."""
import argparse,os,json,math,subprocess,hashlib,itertools
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'11_verification/yoke_c3d10_v10';OUT.mkdir(exist_ok=True)
STEP=ROOT/'09_cad/ankle_v09/RollTransferYoke.step'
EDGES=[(0,1),(1,2),(2,0),(0,3),(1,3),(2,3)]
FIXTURE='face'
GL=np.array([[-1,-1,-1],[1,0,0],[0,1,0],[0,0,1.]])
TRIGL=np.array([[-1,-1],[1,0],[0,1.]])

def tet_shape(l):
    n=np.r_[l*(2*l-1),[4*l[a]*l[b] for a,b in EDGES]]
    grad=np.vstack([(4*l-1)[:,None]*GL,[4*(l[a]*GL[b]+l[b]*GL[a]) for a,b in EDGES]])
    return n,grad

def tri_shape(l):
    edges=[(0,1),(1,2),(2,0)]
    n=np.r_[l*(2*l-1),[4*l[a]*l[b] for a,b in edges]]
    grad=np.vstack([(4*l-1)[:,None]*TRIGL,[4*(l[a]*TRIGL[b]+l[b]*TRIGL[a]) for a,b in edges]])
    return n,grad

def run(cmd,folder,log):
    env=os.environ.copy();env.update(OMP_NUM_THREADS='2',CCX_NPROC_RESULTS='2')
    p=subprocess.run(cmd,cwd=folder,env=env,capture_output=True,text=True,errors='replace',
                     timeout=300,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    log.write_text(p.stdout+'\n'+p.stderr)
    if p.returncode:raise RuntimeError(str(log))

def read_mesh(path):
    lines=path.read_text().splitlines();i=lines.index('$Nodes');count=int(lines[i+1])
    nodes={int(t[0]):np.array(list(map(float,t[1:]))) for t in (x.split() for x in lines[i+2:i+2+count])}
    i=lines.index('$Elements');count=int(lines[i+1]);tets=[];tris=[]
    for line in lines[i+2:i+2+count]:
        t=list(map(int,line.split()));ids=t[3+t[2]:]
        if t[1]==11:
            # Gmsh midside order: 12,23,31,14,34,24; CCX: 12,23,31,14,24,34.
            ids=ids[:8]+[ids[9],ids[8]]
            # Geometry-based check of midside correspondence (curved nodes need not be exact midpoints).
            corners=np.array([nodes[n] for n in ids[:4]])
            mids=np.array([(corners[a]+corners[b])/2 for a,b in EDGES])
            actual=np.array([nodes[n] for n in ids[4:]])
            assert np.array_equal(np.argmin(np.linalg.norm(actual[:,None,:]-mids[None,:,:],axis=2),axis=1),np.arange(6))
            tets.append(ids)
        if t[1]==9:tris.append(ids)
    assert tets and tris
    return nodes,np.array(tets),tris

def boundary(nodes,tris):
    fixed=[n for n,(x,y,z) in nodes.items() if abs(x+9.9)<1e-5 and y*y+(z-45)**2<18.5**2+1e-4]
    if FIXTURE=='bolt_bores':
        holes=[(11.69134295109,51.75),(-11.69134295109,51.75),(0,31.5)]
        fixed=[n for n,(x,y,z) in nodes.items() if -9.9001<=x<=-6.8999 and any(abs(math.hypot(y-hy,z-hz)-1.4)<1e-4 for hy,hz in holes)]
    radial=[n for n,(x,y,z) in nodes.items() if -3.9001<=x<=4.1001 and abs(math.hypot(y,z-45)-4.99)<1e-4]
    quadrature=[(np.array([1/3]*3),.225)]
    for a,b,w in [(.059715871789770,.470142064105115,.132394152788506),(.797426985353087,.101286507323456,.125939180544827)]:
        for idx in range(3):
            l=np.full(3,b);l[idx]=a;quadrature.append((l,w))
    loads={};area=0
    for ids in tris:
        xyz=np.array([nodes[n] for n in ids])
        if max(abs(xyz[:,1]-19.1))>1e-5 or xyz[:,0].min()<6.999 or xyz[:,0].max()>71.001:continue
        for l,w in quadrature:
            shape,grad=tri_shape(l);jac=xyz.T@grad
            da=.5*w*np.linalg.norm(np.cross(jac[:,0],jac[:,1]));area+=da
            for n,weight in zip(ids,shape):loads[n]=loads.get(n,0)+da*weight
    assert len(fixed)>10 and len(radial)>5 and area>1000
    forces={n:-100*w/area for n,w in loads.items() if abs(w)>1e-14}
    assert abs(sum(forces.values())+100)<1e-8
    return fixed,radial,forces,area

def write_deck(folder,nodes,tets,bc):
    fixed,radial,forces,area=bc
    lines=['*HEADING','C3D10 screening; N mm MPa; consistent face Fz=-100N','*NODE,NSET=ALL']
    lines += [f'{n},'+','.join(f'{x:.12g}' for x in p) for n,p in nodes.items()]
    lines+=['*ELEMENT,TYPE=C3D10,ELSET=SOLID']+[f'{i},'+','.join(map(str,t)) for i,t in enumerate(tets,1)]
    for name,ids in [('FIX',fixed),('JOURNAL',radial)]:
        lines+=['*NSET,NSET='+name]+[','.join(map(str,ids[i:i+16])) for i in range(0,len(ids),16)]
    lines+=['*TRANSFORM,NSET=JOURNAL,TYPE=C','0,0,45,1,0,45',
        '*MATERIAL,NAME=AL','*ELASTIC','69000,0.33','*SOLID SECTION,ELSET=SOLID,MATERIAL=AL',
        '*BOUNDARY','FIX,1,3,0','JOURNAL,1,1,0','*STEP','*STATIC','*CLOAD']
    lines += [f'{n},3,{f:.12g}' for n,f in forces.items()]
    lines += ['*NODE PRINT,NSET=ALL,GLOBAL=YES','U','*EL PRINT,ELSET=SOLID','S','*NODE FILE','U','*EL FILE','S','*END STEP']
    (folder/'yoke.inp').write_text('\n'.join(lines)+'\n')
    (folder/'boundary.json').write_text(json.dumps({'fixed':fixed,'radial':radial,'forces':forces,'area_mm2':area},indent=2)+'\n')

def read_solution(path):
    state=None;u={};stress=[]
    for line in path.read_text().splitlines():
        l=line.strip().lower();t=line.split()
        if l.startswith('displacements'):state='u';continue
        if l.startswith('stresses'):state='s';continue
        if not t or not t[0].isdigit():continue
        if state=='u' and len(t)==4:u[int(t[0])]=np.array(list(map(float,t[1:])))
        if state=='s' and len(t)==8:
            sx,sy,sz,xy,xz,yz=map(float,t[2:])
            vm=math.sqrt(.5*((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)+3*(xy*xy+xz*xz+yz*yz))
            stress.append((int(t[0]),int(t[1]),vm))
    assert u and stress
    return u,stress

def integrate_internal(nodes,tets,u):
    xyz=np.array([[nodes[int(n)] for n in t] for t in tets])
    ue=np.array([[u[int(n)] for n in t] for t in tets]);fe=np.zeros_like(ue)
    a=.5854101966249685;b=.1381966011250105
    E=69000.;nu=.33;mu=E/(2*(1+nu));lam=E*nu/((1+nu)*(1-2*nu))
    minimum=1e9;volumes=np.zeros(len(tets))
    for i in range(4):
        l=np.full(4,b);l[i]=a;n,grad=tet_shape(l)
        jac=np.einsum('eni,nj->eij',xyz,grad);det=np.linalg.det(jac)
        minimum=min(minimum,float(det.min()));assert np.all(det>0),'Nonpositive element Jacobian'
        gx=np.einsum('nj,ejk->enk',grad,np.linalg.inv(jac))
        du=np.einsum('eni,enj->eij',ue,gx);eps=.5*(du+du.transpose(0,2,1))
        sigma=2*mu*eps+lam*np.trace(eps,axis1=1,axis2=2)[:,None,None]*np.eye(3)
        fe+=np.einsum('eij,enj,e->eni',sigma,gx,det/24);volumes+=det/24
    fint=np.zeros((max(nodes)+1,3));np.add.at(fint,tets.ravel(),fe.reshape(-1,3))
    return fint,minimum,float(volumes.sum())

def main():
    global FIXTURE,OUT,STEP
    parser=argparse.ArgumentParser();parser.add_argument('--sizes',type=float,nargs='+',default=[3.5,2.5,1.8])
    parser.add_argument('--fixture',choices=['face','bolt_bores'],default='face')
    parser.add_argument('--geometry',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();FIXTURE=args.fixture
    if FIXTURE=='bolt_bores':OUT=ROOT/'11_verification/yoke_c3d10_bore_fixture_v10';OUT.mkdir(exist_ok=True)
    if args.geometry:STEP=args.geometry.resolve()
    if args.output:OUT=args.output.resolve();OUT.mkdir(exist_ok=True)
    results=[]
    for h in args.sizes:
        folder=OUT/f'mesh_{h}';folder.mkdir(exist_ok=True)
        (folder/'mesh.geo').write_text(f'SetFactory("OpenCASCADE");\nv()=ShapeFromFile("{STEP.as_posix()}");\nMesh.MeshSizeMin={h}; Mesh.MeshSizeMax={h};\nMesh.ElementOrder=2; Mesh.SecondOrderLinear=0; Mesh.HighOrderOptimize=1; Mesh.MshFileVersion=2.2; Mesh.SaveAll=1;\n')
        print('Meshing quadratic',h,flush=True)
        run([r'G:\freeCAD\bin\gmsh.exe','mesh.geo','-3','-o','mesh.msh','-nt','2'],folder,folder/'gmsh.log')
        nodes,tets,tris=read_mesh(folder/'mesh.msh');bc=boundary(nodes,tris);write_deck(folder,nodes,tets,bc)
        print('Solving',h,len(nodes),'nodes',len(tets),'C3D10',flush=True)
        run([r'G:\freeCAD\bin\ccx.exe','-i','yoke'],folder,folder/'solver.log')
        u,stress=read_solution(folder/'yoke.dat');fint,minjac,volume=integrate_internal(nodes,tets,u)
        fix,radial,forces,area=bc;support=fix+radial
        force=fint[support].sum(axis=0)
        moment=sum((np.cross(nodes[n],fint[n]) for n in support),np.zeros(3))
        appliedmoment=sum((np.cross(nodes[n],[0,0,f]) for n,f in forces.items()),np.zeros(3))
        ein=.5*sum(u[n]@fint[n] for n in nodes);eout=.5*sum(f*u[n][2] for n,f in forces.items())
        peak=max(stress,key=lambda x:x[2]);norm={n:float(np.linalg.norm(v)) for n,v in u.items()}
        row={'h_mm':h,'fixture':FIXTURE,'nodes':len(nodes),'elements':len(tets),'min_gauss_jacobian':minjac,'mesh_volume_mm3':volume,
            'max_u_mm':max(norm.values()),'load_weighted_uz_mm':sum(-f*u[n][2]/100 for n,f in forces.items()),
            'peak_vm_MPa':peak[2],'peak_element':peak[0],'force_residual_N':float(np.linalg.norm(force+[0,0,-100])),
            'moment_residual_Nmm':float(np.linalg.norm(moment+appliedmoment)),
            'energy_relative_error':float(abs(ein-eout)/abs(eout)),'load_area_mm2':area}
        (folder/'summary.json').write_text(json.dumps(row,indent=2)+'\n');results.append(row)
        print(json.dumps(row),flush=True)
    name='summary.json' if args.sizes==[3.5,2.5,1.8] else 'summary_additional.json'
    (OUT/name).write_text(json.dumps({'status':'linear_local_screen_not_release','geometry_sha256':hashlib.sha256(STEP.read_bytes()).hexdigest(),'results':results},indent=2)+'\n')
if __name__=='__main__':main()
