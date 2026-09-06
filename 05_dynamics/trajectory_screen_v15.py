"""Planar rigid-body inverse dynamics by virtual power; not fullbody simulation."""
import json, math, hashlib
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '05_dynamics/trajectory_v15'
PFILE = ROOT / '03_system/layout_load_inputs_v15.json'
G = 9.81

def kin(q, dq, ddq, lengths, segment_index, fraction):
    """Position/Jacobian/velocity/acceleration of a point on segment i, x forward/z up."""
    n = len(q); theta=np.cumsum(q); w=np.cumsum(dq); a=np.cumsum(ddq)
    pos=np.zeros(2); vel=pos.copy(); acc=pos.copy(); jac=np.zeros((2,n))
    for k in range(segment_index+1):
        r=lengths[k]*(fraction if k==segment_index else 1)
        v=r*np.array([math.sin(theta[k]), math.cos(theta[k])])
        tangent=r*np.array([math.cos(theta[k]), -math.sin(theta[k])])
        pos+=v; vel+=tangent*w[k]; acc+=tangent*a[k]-v*w[k]**2
        jac[:,:k+1]+=tangent[:,None]
    return pos, jac, vel, acc

def dynamics(q,dq,ddq,lengths,masses,payload=0,com_offsets=None):
    n=len(q); tau=np.zeros(n); F=np.zeros(2); moment=0.; energy=0.; dp=0.
    theta_dot=np.cumsum(dq); theta_ddot=np.cumsum(ddq)
    for i,m in enumerate(masses):
        pos,jac,v,a=kin(q,dq,ddq,lengths,i,.5)
        if com_offsets is not None:
            theta=sum(q[:i+1]);offset=com_offsets[i]*np.array([math.cos(theta),-math.sin(theta)])
            tangent=com_offsets[i]*np.array([-math.sin(theta),-math.cos(theta)])
            pos+=offset;v+=tangent*theta_dot[i];a+=tangent*theta_ddot[i]-offset*theta_dot[i]**2
            jac[:,:i+1]+=tangent[:,None]
        inertia=m*lengths[i]**2/12
        f=m*(a+np.array([0,G])); tau+=jac.T@f
        tau[:i+1]+=inertia*theta_ddot[i]
        F+=f; moment+=pos[1]*f[0]-pos[0]*f[1]+inertia*theta_ddot[i]
        energy+=.5*m*(v@v)+m*G*pos[1]+.5*inertia*theta_dot[i]**2
        dp+=f@v+inertia*theta_ddot[i]*theta_dot[i]
    if payload:
        pos,jac,v,a=kin(q,dq,ddq,lengths,n-1,1)
        f=payload*(a+np.array([0,G])); tau+=jac.T@f; F+=f
        moment+=pos[1]*f[0]-pos[0]*f[1]
        energy+=.5*payload*(v@v)+payload*G*pos[1]; dp+=f@v
    return tau,F,moment,energy,float(tau@dq-dp)

def trajectory(start,end,T,N):
    t=np.linspace(0,T,N); u=t/T; delta=np.asarray(end)-start
    h=10*u**3-15*u**4+6*u**5
    hd=(30*u**2-60*u**3+30*u**4)/T
    hdd=(60*u-180*u**2+120*u**3)/T**2
    return t, np.asarray(start)+h[:,None]*delta, hd[:,None]*delta,hdd[:,None]*delta

def analyse(name,t,q,dq,ddq,lengths,masses,payload,driver,footmass=0,leg=None):
    rows=[]; energies=[]; residuals=[]
    for time,qi,vi,ai in zip(t,q,dq,ddq):
        offsets=[0,0,leg['upper_body_com_forward_offset_m']] if leg else None
        tau,F,M,E,res=dynamics(qi,vi,ai,lengths,masses,payload,offsets)
        if leg:M+=leg['ankle_axis_height_m']*F[0]
        F[1]+=footmass*G
        cop=-M/F[1] if F[1]>1e-9 else None
        rows.append({'time_s':float(time),'q_rad':qi.tolist(),'dq_rad_s':vi.tolist(),'ddq_rad_s2':ai.tolist(),
                     'tau_Nm':tau.tolist(),'required_reaction_xz_N':F.tolist(),'required_CoP_x_m':cop,
                     'joint_power_W':(tau*vi).tolist()})
        energies.append(E); residuals.append(res)
    tau=np.array([r['tau_Nm'] for r in rows]); dt=np.diff(t)
    power=np.sum(tau*dq,axis=1)
    work=float(np.sum((power[1:]+power[:-1])*dt/2))
    deltaE=energies[-1]-energies[0]
    def duration(mask):return float(np.sum((mask[1:].astype(float)+mask[:-1].astype(float))*dt/2))
    bounds=[]
    for j in range(len(masses)):
        limit=driver.get('peak_torque_Nm',driver.get('stall_torque_Nm'))
        speedlim=driver['no_load_rpm']*2*math.pi/60
        ix=int(np.argmax(np.abs(tau[:,j])))
        bounds.append({'joint_index':j,'peak_abs_tau_Nm':float(np.max(np.abs(tau[:,j]))),
          'dq_at_peak_tau_rad_s':float(dq[ix,j]),'time_at_peak_tau_s':float(t[ix]),
          'peak_abs_dq_rad_s':float(np.max(np.abs(dq[:,j]))),
          'single_motion_rms_tau_Nm':float(np.sqrt(np.sum((tau[1:,j]**2+tau[:-1,j]**2)*dt/2)/t[-1])),
          'sampled_duration_over_torque_bound_s':duration(np.abs(tau[:,j])>limit),
          'sampled_duration_over_no_load_speed_s':duration(np.abs(dq[:,j])>speedlim),
          'sampled_duration_over_rated_torque_s':duration(np.abs(tau[:,j])>driver['rated_torque_Nm']) if 'rated_torque_Nm' in driver else None,
          'motor_feasible':None})
    contact=None
    if leg:
        ok=[r['required_reaction_xz_N'][1]>0 and r['required_CoP_x_m'] is not None and
            abs(r['required_CoP_x_m'])<=leg['sole_half_length_m']-leg['sole_margin_m'] and
            abs(r['required_reaction_xz_N'][0])<=leg['assumed_ground_mu']*r['required_reaction_xz_N'][1] for r in rows]
        contact={'all_sampled_planar_necessary_conditions':all(ok),'invalid_sample_count':len(ok)-sum(ok),
                 'minimum_Fz_N':min(r['required_reaction_xz_N'][1] for r in rows),
                 'max_abs_CoP_m':max(abs(r['required_CoP_x_m']) for r in rows if r['required_CoP_x_m'] is not None),
                 'not_dynamic_stability_proof':True}
    result={'name':name,'duration_s':float(t[-1]),'masses_kg':list(masses),'tip_payload_kg':payload,
            'joint_summaries':bounds,'contact':contact,'work_J':work,'energy_change_J':deltaE,
            'work_energy_residual_J':work-deltaE,'max_virtual_power_residual_W':float(max(abs(x) for x in residuals)),
            'rows':rows,'task_verified':False}
    (OUT/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return {k:v for k,v in result.items() if k!='rows'}

def main():
    p=json.loads(PFILE.read_text(encoding='utf-8'));OUT.mkdir(exist_ok=True)
    allcases=[];leg=p['leg'];L=leg['link_lengths_m'];N=leg['samples']
    for case in p['cases']:
        m=[case['shin_mass_each_kg'],case['thigh_mass_each_kg'],case['robot_mass_kg']/2-case['foot_mass_each_kg']-case['shin_mass_each_kg']-case['thigh_mass_each_kg']]
        assert all(x>0 for x in m)
        assert abs(2*(sum(m)+case['foot_mass_each_kg'])-case['robot_mass_kg'])<1e-10
        start=np.deg2rad([leg['crouch_alpha_deg'],-2*leg['crouch_alpha_deg'],leg['crouch_alpha_deg']])
        end=np.deg2rad([leg['standing_alpha_deg'],-2*leg['standing_alpha_deg'],leg['standing_alpha_deg']])
        for T in leg['durations_s']:
            t,q,dq,ddq=trajectory(start,end,T,N)
            allcases.append(analyse(f"leg_{case['id']}_{T:g}s",t,q,dq,ddq,L,m,0,p['driver_references'][case['pitch_driver']],case['foot_mass_each_kg'],leg))
    arm=p['arm']
    for T in arm['durations_s']:
        t,q,dq,ddq=trajectory(np.deg2rad(arm['q_start_deg']),np.deg2rad(arm['q_end_deg']),T,N)
        allcases.append(analyse(f'arm_lift_{T:g}s',t,q,dq,ddq,arm['link_lengths_m'],arm['link_masses_kg'],arm['half_bottle_mass_kg'],p['driver_references']['XM430']))
    # Fixed-wrist support ramp: contact demand, no implied fullbody getup motion.
    support=[];s=p['support']
    for case in p['cases']:
        times=np.linspace(0,s['ramp_duration_s']+s['hold_duration_s'],501)
        u=np.minimum(times/s['ramp_duration_s'],1);fraction=10*u**3-15*u**4+6*u**5
        force=fraction*s['weight_fraction_on_two_hands']*case['robot_mass_kg']*G/2
        support.append({'case':case['id'],'time_s':times.tolist(),'each_hand_force_N':force.tolist(),
                        'wrist_torque_Nm':(force*s['lever_m']).tolist(),'max_wrist_torque_Nm':float(max(force*s['lever_m'])),
                        'continuous_motor_feasible':None})
    summary={'version':'v0.15','input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),
             'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'cases':allcases,'support_ramps':support,
             'limitations':['Fixed-foot planar model and fixed-shoulder arm; not coupled 26DOF dynamics',
             'Uniform rod inertia and lumped mass approximations; rotor reflected inertia/friction/elasticity missing',
             'Torque and speed bounds are necessary-only, not a rectangular allowable working envelope',
             'Rest-to-rest extension has zero end speed and does not achieve takeoff',
             'Sampled threshold durations and contact checks need time refinement; no thermal validation'],
             'required_unverified':p['required_unverified']}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    for r in allcases:print(r['name'],'peak torques', [round(j['peak_abs_tau_Nm'],3) for j in r['joint_summaries']],
                           'peak speeds',[round(j['peak_abs_dq_rad_s'],3) for j in r['joint_summaries']],
                           'contact',r['contact'] and r['contact']['all_sampled_planar_necessary_conditions'])
if __name__=='__main__':main()
