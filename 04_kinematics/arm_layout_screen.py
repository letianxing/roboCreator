"""Idealized arm position + approach-direction IK comparison; no collision/dynamics."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def rot(axis, q):
    v=np.eye(3)['xyz'.index(axis)]
    x,y,z=v
    k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+np.sin(q)*k+(1-np.cos(q))*(k@k)

def fk(q, kind, shoulder_z=.295):
    shoulder_axes = 'yx' if kind==3 else 'yxz'
    r=np.eye(3)
    p=np.array([0.,.08,shoulder_z])
    for axis,angle in zip(shoulder_axes,q):
        r=r@rot(axis,angle)
    p=p+r@np.array([0.,0.,-.08])
    r=r@rot('y',q[len(shoulder_axes)])
    p=p+r@np.array([0.,0.,-.08])
    if kind==5:
        r=r@rot('x',q[-1])
    approach=r@np.array([0.,0.,-1.])
    p=p+.04*approach
    return p,approach

def solve(target, direction, kind, orient, shoulder_z):
    rng=np.random.default_rng(20260906)
    # Broad analytical limits, not manufactured ROM or safe control limits.
    lo=np.full(kind,-np.pi)
    hi=np.full(kind,np.pi)
    elbow=2 if kind==3 else 3
    lo[elbow],hi[elbow]=0.,np.deg2rad(150)
    def residual(q):
        p,d=fk(q,kind,shoulder_z)
        return np.r_[p-target, .08*(d-direction)] if orient else p-target
    best=None
    for seed in range(12):
        q=np.clip(np.zeros(kind) if seed==0 else rng.uniform(lo,hi),lo,hi)
        for _ in range(160):
            e=residual(q)
            j=np.column_stack([(residual(q+np.eye(kind)[i]*1e-5)-e)/1e-5 for i in range(kind)])
            dq=-np.linalg.solve(j.T@j+1e-5*np.eye(kind),j.T@e)
            q=np.clip(q+np.clip(dq,-.2,.2),lo,hi)
            if np.linalg.norm(e)<1e-5:
                break
        score=float(np.linalg.norm(residual(q)))
        if best is None or score<best[0]:
            best=(score,q.copy())
    q=best[1]
    p,d=fk(q,kind,shoulder_z)
    pos=float(np.linalg.norm(p-target))
    ang=float(np.degrees(np.arccos(np.clip(d@direction,-1,1))))
    return dict(q_deg=np.degrees(q).tolist(),position_error_mm=1000*pos,
        approach_error_deg=ang,screen_pass=pos<=.002 and (not orient or ang<=5),
        interpretation='numerical candidate only; failure is not proof of infeasibility')

def main():
    tasks=[('table_bottle',[.10,.04,.13],[0.,-1.,0.],.295),
           ('table_bottle_lowered_body',[.10,.04,.13],[0.,-1.,0.],.255),
           ('handover',[.15,.04,.24],[0.,-1.,0.],.295),
           ('forward_support',[.12,.10,.18],[0.,0.,-1.],.295)]
    results=[]
    for kind in (3,4,5):
        for name,point,direction,shoulder_z in tasks:
            for orient in (False,True):
                row=dict(arm_dof=kind,task=name,position_and_direction=orient,shoulder_z_m=shoulder_z)
                row.update(solve(np.array(point),np.array(direction),kind,orient,shoulder_z))
                results.append(row)
    data=dict(status='idealized_left_arm_only',seed=20260906,
        lengths_m=[.08,.08,.04],shoulder_xyz_m=[0,.08,.295],
        limitations=['No self-collision or motor envelope','No loads or full-body balance',
            'Approach direction is not full hand orientation','Support is only a kinematic proxy, not a get-up trajectory',
            'Joint axes/limits assumed; left arm only; no manufactured assembly'],results=results)
    path=ROOT/'04_kinematics'/'arm_layout_results_v02.json'
    path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    for row in results:
        print(f"arm={row['arm_dof']} task={row['task']} direction={row['position_and_direction']} "
              f"pos_mm={row['position_error_mm']:.2f} dir_deg={row['approach_error_deg']:.2f} pass={row['screen_pass']}")

if __name__=='__main__':
    main()
