"""Palm IK with cuff-clearance penalty; returns distinct branches for exact CAD screening."""
import math
import numpy as np
from scipy.optimize import least_squares
from search_recovery_v21 import old,rot

def fk(q,row):
    _,_,nodes=old.arm.fk(q[:4],row['h_mm']/1000,math.radians(row['beta_deg']),28,180)
    W=nodes['wrist']+np.array([row['hip_x_mm']/1000,.006,0]);R=nodes['forearm_rotation'];P=R@rot('x',q[4])@rot('y',q[5])
    point=W+P@np.array([.0245,0,-.028]);normal=P@np.array([1,0,0])
    corners=[W+R@np.array([.025,sy*.020,.012+sx*.010]) for sy in (-1,1) for sx in (-1,1)]
    return point,normal,min(c[2] for c in corners)

def candidates(row,refine=False):
    lo=np.deg2rad([-160,-120,-160,0,-130,-160]);hi=np.deg2rad([160,120,160,150,130,160]);out=[]
    seeds=[[-40,0,-145,65,0,30],[-60,40,130,35,40,80],[-80,-15,-90,45,-40,65],[-75,35,90,20,30,110],[-110,20,20,30,50,100]]
    if refine:seeds += [[-30,0,90,1,0,90],[-30,0,-90,1,0,90],[-30,10,0,10,0,90]]
    for dx in ((20,40,60) if refine else (40,80,120)):
        for y in ((71,91,111,131) if refine=='midpitch' else (51,71,91,111) if refine else (111,141,171)):
            target=np.array([row['hip_x_mm']+dx,y,0])/1000
            def res(q):
                p,n,z=fk(q,row);return np.r_[p-target,.08*(n-[0,0,-1]),3*min(0,z-.001)]
            for seed in seeds:
                r=least_squares(res,np.deg2rad(seed),bounds=(lo,hi),max_nfev=100,ftol=1e-10,xtol=1e-10,gtol=1e-10)
                p,n,z=fk(r.x,row);pe=float(np.linalg.norm(p-target)*1000);ne=math.degrees(math.acos(np.clip(n@np.array([0,0,-1]),-1,1)))
                if pe>.05 or ne>.05 or z<.00099:continue
                q=np.rad2deg(r.x)
                if any(np.linalg.norm(q-np.array(v['q_deg']))<3 for v in out):continue
                out.append({'q_deg':q.tolist(),'target_mm':(target*1000).tolist(),'position_error_mm':pe,'normal_error_deg':ne,'cuff_corner_min_z_mm':z*1000})
    return out
