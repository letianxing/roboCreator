"""Whole-body point-mass and envelope audit; not a rigid-body dynamics solver."""
import importlib.util
import itertools
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('arm',Path(__file__).with_name('arm_layout_screen.py'))
arm=importlib.util.module_from_spec(spec); spec.loader.exec_module(arm)

PROFILES={
 'servo_reference':{'main_mass':.055,'small_mass':.055,'main_diameter':.0452,'main_length':.035},
 'ak40_xc330_reference':{'main_mass':.190,'small_mass':.023,'main_diameter':.053,'main_length':.0402}
}

def pose(profile,case,q):
    masses=[]; motors=[]; arm_chains=[]
    def item(name,m,p):
        masses.append({'name':name,'mass_kg':m,'xyz_m':list(map(float,p))})
    def motor(name,p,small=False):
        m=profile['small_mass' if small else 'main_mass']
        item(name,m,p)
        motors.append({'name':name,'xyz_m':list(map(float,p)),'small':small})
    h=.165 if case=='pickup_crouch' else .205
    knee_x=float(np.sqrt(.09**2-((h-.025)/2)**2))
    knees=[]
    for side in (1,-1):
        y=side*.055
        hip=np.array([0,y,h]); knee=np.array([knee_x,y,(h+.025)/2]); ankle=np.array([0,y,.025])
        knees.append(knee.tolist())
        for axis in ('yaw','roll','pitch'): motor(f'hip_{side}_{axis}',hip)
        motor(f'knee_{side}',knee)
        for axis in ('pitch','roll'): motor(f'ankle_{side}_{axis}',ankle)
        item(f'thigh_frame_{side}',.02,(hip+knee)/2)
        item(f'shin_frame_{side}',.02,(knee+ankle)/2)
        item(f'foot_{side}',.06,[0,y,.0125])
    for side in (1,-1):
        # Right arm mirrored geometrically. Right-joint signs remain to be defined.
        mirror=np.diag([1,side,1])
        p=np.array([0,.08,h+.09]); r=np.eye(3)
        chain=[]
        for i,axis in enumerate('yxz'):
            a=r@np.eye(3)['xyz'.index(axis)]
            chain.append((p.copy(),a.copy()))
            motor(f'shoulder_{side}_{axis}',mirror@p)
            r=r@arm.rot(axis,q[i])
        p=p+r@np.array([0,0,-.08])
        chain.append((p.copy(),r@np.array([0,1,0])))
        motor(f'elbow_{side}',mirror@p)
        r=r@arm.rot('y',q[3]); p=p+r@np.array([0,0,-.08])
        chain.append((p.copy(),r@np.array([1,0,0])))
        motor(f'wrist_{side}',mirror@p,True)
        r=r@arm.rot('x',q[4]); tip=p+r@np.array([0,0,-.04])
        motor(f'gripper_drive_{side}',mirror@tip,True)
        item(f'gripper_structure_{side}',.04,mirror@tip)
        arm_chains.append((side,chain,tip))
    for axis in ('yaw','pitch'):motor(f'head_{axis}',[0,0,h+.095],True)
    item('torso_frame',.12,[0,0,h+.045])
    item('battery',.22,[0,0,h+.02])
    item('compute_power',.18,[0,0,h+.06])
    item('audio',.08,[0,0,h+.14])
    item('vision',.13,[.025,0,h+.13])
    item('fasteners_wiring',.12,[0,0,h])
    item('costume_contingency',.15,[0,0,h+.06])
    robot_mass=sum(a['mass_kg'] for a in masses)
    # Bottle weight carried by robot only for the handover case.
    if case=='handover':item('bottle',.58,[.15,0,.24])
    total=sum(a['mass_kg'] for a in masses)
    com=sum(a['mass_kg']*np.array(a['xyz_m']) for a in masses)/total
    # Coplanar double-support convex hull of 120 x 55 mm feet centered at y=+/-55mm.
    margin=min(.06-abs(com[0]),.0825-abs(com[1]))
    overlaps=[]
    for a,b in itertools.combinations(motors,2):
        if np.linalg.norm(np.array(a['xyz_m'])-np.array(b['xyz_m']))<1e-9:
            overlaps.append([a['name'],b['name']])
    return dict(case=case,robot_mass_kg=robot_mass,total_mass_kg=total,
        com_xyz_m=com.tolist(),static_projection_margin_m=float(margin),
        nominal_margin_15mm_pass=bool(margin>=.015),knees_xyz_m=knees,
        coincident_motor_center_pairs=overlaps,motor_count=len(motors),point_masses=masses,
        caveat='Static necessary condition only. Motor center conflicts make this layout non-manufacturable.')

def main():
    ik=json.loads((ROOT/'04_kinematics/arm_layout_results_v02.json').read_text())
    def q_for(task):
        row=next(r for r in ik['results'] if r['arm_dof']==5 and r['task']==task and r['position_and_direction'])
        assert row['screen_pass']
        return np.deg2rad(row['q_deg'])
    qs={'standing':np.zeros(5),'pickup_crouch':q_for('table_bottle_lowered_body'),
        'handover':q_for('handover')}
    result={'status':'audit_not_validated_robot','profiles':PROFILES,'cases':{}}
    for name,profile in PROFILES.items():
        result['cases'][name]=[pose(profile,case,q) for case,q in qs.items()]
    result['assumptions']=[
      '400mm upright envelope, ideal coaxial shoulder/hip joints, zero motor mounting offset',
      '20 main and 6 small actuators. AK40 V3=190g/53mm diameter/40.2mm length; XC330=23g',
      'Non-actuator mass allowances total 1.28kg; not CAD or measured masses',
      'Point masses give static COM only, no inertia or force distribution',
      '120x55mm assumed feet, centers 110mm apart, flat coplanar full-foot contact',
      'Arms mirrored; right-arm hardware signs and collisions unresolved',
      'Pickup bottle is still table-supported, handover fully robot-supported',
      'Coincident motor centers are real packaging conflicts for separate solid modules; not all collisions are checked']
    (ROOT/'04_kinematics/whole_body_results_v03.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,cases in result['cases'].items():
        for c in cases:
            print(name,c['case'],'mass',round(c['total_mass_kg'],3),'COM_mm',
                  np.round(np.array(c['com_xyz_m'])*1000,2).tolist(),
                  'margin_mm',round(c['static_projection_margin_m']*1000,2),
                  'motor_conflicts',len(c['coincident_motor_center_pairs']))

if __name__=='__main__': main()
