"""Correct 3D left-leg local offsets; legacy right leg/body stays sagittal."""
import FreeCAD as A,Part,json,math
from pathlib import Path
from route_geometry_v20 import load
ROOT=Path(__file__).resolve().parents[1];V=A.Vector

class Geometry:
    def __init__(self):
        self.mod=load();self.env=self.mod.env
        self.out=ROOT/'09_cad/body_v21';self.out.mkdir(exist_ok=True);self.env['OUT']=self.out

    def arm_frames(self,h,beta,x,q):
        B=A.Rotation(V(0,1,0),beta);S=V(x,0,h)+B.multVec(V(0,81,80));R=B;frames={}
        for i,axis in enumerate([(0,1,0),(1,0,0),(0,0,1)]):
            frames['ShoulderMotor'+str(i)]=A.Placement(S,R);R=R.multiply(A.Rotation(V(*axis),q[i]))
        frames['UpperArm']=A.Placement(S,R);E=S+R.multVec(V(0,0,-80));frames['ElbowMotor']=A.Placement(E,R)
        R=R.multiply(A.Rotation(V(0,1,0),q[3]));W=E+R.multVec(V(0,0,-85));fore=A.Placement(W,R)
        for n in ('ForearmLoadFrame','GauntletPrimaryPad','WristMotorEnvelope0','WristMotorEnvelope1'):frames[n]=fore
        hand=A.Placement(W,R.multiply(A.Rotation(V(1,0,0),q[4])).multiply(A.Rotation(V(0,1,0),q[5])))
        for n in ('HandAndPalm','GripperMotor'):frames[n]=hand
        return frames,fore.multVec(V(25,0,12)),hand.multVec(V(24.5,0,-28))

    def prepare_arms(self,row):
        q=[0]*6;whole,info,*_=self.mod.build_geometry({'pose':row['right_pose'],'q':q})
        frames,*_=self.arm_frames(row['h_mm'],row['beta_deg'],row['hip_x_mm'],q)
        self.arm_native={n:self.env['placed'](whole['L_'+n],T.inverse()) for n,T in frames.items()}
        self.arm_meta={n:info['L_'+n] for n in frames}

    def arms(self,row,q,side=1):
        if not hasattr(self,'arm_native'):self.prepare_arms(row)
        frames,cu,pa=self.arm_frames(row['h_mm'],row['beta_deg'],row['hip_x_mm'],q);parts={};meta={};tag='L' if side==1 else 'R'
        for n,T in frames.items():
            s=self.env['placed'](self.arm_native[n],T)
            parts[tag+'_'+n]=s if side==1 else self.env['mirror'](s);meta[tag+'_'+n]={**self.arm_meta[n],'group':tag+'_arm'}
        return parts,meta,[cu.x,side*cu.y,cu.z],[pa.x,side*pa.y,pa.z]

    def fixed(self,row):
        B=A.Rotation(V(0,1,0),row['beta_deg']);T=A.Placement(V(row['hip_x_mm'],0,row['h_mm']),B)
        if not hasattr(self,'fixed_native'):
            whole,info,*_=self.mod.build_geometry({'pose':row['right_pose'],'q':[0]*6})
            names=[n for n in whole if not info[n]['group'].endswith('_leg') and
                   (not info[n]['group'].endswith('_arm') or n.endswith('ShoulderMotor0'))]
            self.fixed_native={n:self.env['placed'](whole[n],T.inverse()) for n in names};self.fixed_meta={n:info[n] for n in names}
        p={n:self.env['placed'](s,T) for n,s in self.fixed_native.items()};m=dict(self.fixed_meta)
        if row.get('power_shift_body_mm'):p['power'].translate(B.multVec(V(*row['power_shift_body_mm'])))
        lp,lm,_,_=self.leg(row['right_pose'])
        for n,s in lp.items():
            s=self.env['mirror'](s);s.translate(V(0,-60,0));p['R_'+n]=s;m['R_'+n]={**lm[n],'group':'R_leg'}
        return p,m

    def boxes(self,parts):
        # Conservative transformed native AABBs. Never reject a pair using approximate meshes.
        if not hasattr(self,'box_cache'):self.box_cache={}
        out={}
        for n,s in parts.items():
            local=s.copy();T=local.Placement;local.Placement=A.Placement();key=local.hashCode()
            bucket=self.box_cache.setdefault(key,[]);found=next((bb for shape,bb in bucket if shape.isSame(local)),None)
            if found is None:
                found=local.BoundBox;bucket.append((local,found))
            b=A.BoundBox()
            for x in (found.XMin,found.XMax):
                for y in (found.YMin,found.YMax):
                    for z in (found.ZMin,found.ZMax):b.add(T.multVec(V(x,y,z)))
            out[n]=b
        return out

    def leg(self,pose):
        e=self.env['narrow'];parts,meta,joints,frames=e['build'](pose,35)
        # Original narrow wrapper moved ThighRail1 in WORLD Y. Use THIGH Y instead.
        Rt=frames['shin'].Rotation.multiply(A.Rotation(V(0,1,0),-pose['q_deg'][3]))
        parts['ThighRail1'].translate(Rt.multVec(V(0,-10,0)))
        K=V(*joints[3]['origin_mm']);T=A.Placement(K,Rt)
        holder=e['placed'](self.mod.holder(V(0,0,0)),T)
        pad=e['placed'](self.mod.pad(V(0,0,0)),T)
        before=parts['ThighRail1'];joined=before.fuse(holder).removeSplitter()
        meta['ThighRail1']['mass_kg']+=(joined.Volume-before.Volume)*2.7e-6;parts['ThighRail1']=joined
        parts['KneePad']=pad;meta['KneePad']={'mass_kg':pad.Volume*self.mod.cfg['knee_pad']['density_kg_mm3'],'role':'pad','rigid_group':'thigh'}
        return parts,meta,joints,frames

    def body(self,row,armq,rightq=None):
        assert row['right_pose']['q_deg'][0]==row['right_pose']['q_deg'][1]==row['right_pose']['q_deg'][4]==0
        scene={'name':'candidate','pose':row['right_pose'],'q':armq,'right_q':rightq or armq}
        whole,info,rj,kn,cu,pa=self.mod.build_geometry(scene)
        lp,lm,lj,frames=self.leg(row['left_pose'])
        # Compare common HIP-YAW root, not moving hip-pitch joint, for nonzero hip roll/yaw.
        root=frames['body'];assert (root.Base-V(*rj[0]['origin_mm'])).Length<.011
        B=A.Rotation(V(0,1,0),row['beta_deg']);assert abs(root.Rotation.multiply(B.inverted()).Angle)<math.radians(.011)
        for n in list(whole):
            if info[n]['group']=='L_leg':whole.pop(n);info.pop(n)
        for n,s in lp.items():
            s=s.copy();s.translate(V(0,60,0));whole['L_'+n]=s;info['L_'+n]={**lm[n],'group':'L_leg'}
        return whole,info,{'left':lj,'right':rj},kn,cu,pa
