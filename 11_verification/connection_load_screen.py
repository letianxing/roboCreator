"""Bolt-group and ideal two-support bearing sensitivity; no component approval."""
import json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'11_verification/yoke_fea_reinforced_v09'
OUT.mkdir(exist_ok=True)
g=9.81;rows=[]
for mass in (5.,7.):
    for feet in (1,2):
        for factor in (1.,1+.1/.03):
            rows.append({'assumed_mass_kg':mass,'support_feet':feet,'equivalent_force_factor':factor,
                         'per_foot_vertical_N':mass*g*factor/feet})
# Only shows dependence on unknown internal bearing position; both supports ideal pins.
bearing=[]
for inner_x in (-30.,-20.,-9.9):
    outer_x=.1;load_x=39.;force=100.
    rb=force*(load_x-inner_x)/(outer_x-inner_x);ra=force-rb
    bearing.append({'assumed_internal_support_x_mm':inner_x,'external_bearing_x_mm':outer_x,
        'force_N':force,'external_reaction_N':rb,'internal_reaction_N':ra})
points=[(11.69134295109,6.75),(-11.69134295109,6.75),(0,-13.5)]
sr2=sum(y*y+z*z for y,z in points);bolt=[]
for y,z in points:
    fy=-4100*z/sr2;fz=4100*y/sr2-100/3
    bolt.append({'y_mm':y,'z_mm':z,'shear_y_N':fy,'shear_z_N':fz,'resultant_N':math.hypot(fy,fz)})
vols={}
for variant in ('v08','v09'):
    r=json.loads((ROOT/f'09_cad/ankle_{variant}/verification.json').read_text())
    metal=['FootPlate','RollStatorMount','RollBearingCarrier','RollBearingCap','RollTransferYoke','PitchOutputShankBracket']
    vols[variant]={'yoke_mass_kg_at_2700_density':r['part_volumes_mm3']['RollTransferYoke']*2.7e-6,
                   'six_metal_parts_mass_kg':sum(r['part_volumes_mm3'][n] for n in metal)*2.7e-6}
result={'status':'sensitivity_only','load_cases':rows,'ideal_pin_bearing_cases':bearing,
    'bolt_group_combined_screen':{'vertical_force_N':100,'axis_torque_Nm':4.1,'bolt_count':3,'pcd_mm':27,
        'bolts':bolt,'maximum_shear_N':max(x['resultant_N'] for x in bolt)},'mass':vols,
    'limitations':['5/7kg are assumed, not measured robot masses',
        'Landing factor from simple vertical energy model is displacement-average, not peak',
        'Bearing support coordinates inside motor unknown; pin reactions cannot replace real bearing/contact model',
        'Bolt forces assume equal shear stiffness and no interface friction; no preload/prying/axial load or thread stripping check',
        '4.1Nm motor peak combined with 100N is an independent stress scenario, not a verified actual simultaneous load',
        'Material density assumed 2700kg/m3; strength and stock certificate not selected']}
(OUT/'connection_load_screen.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
