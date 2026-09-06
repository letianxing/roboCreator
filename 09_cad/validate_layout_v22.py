"""Validate selected fixed layout across local motion samples plus a standing regression."""
import json,hashlib,math
from geometry_v22 import *
search=json.loads((OUT/'layout_clearance_2.json').read_text());assert search['complete'] and search['joint_layouts']
layout=search['joint_layouts'][0]['layout'];changed=set(layout);nominal=search['nominal_clearance_mm']
selected={'stage':'v0.22','translations_relative_v21_body_mm':layout,'nominal_clearance_mm':nominal,'selection_rule':'Minimum total rigid translation among2mm-clearance survivors','source':'09_cad/body_v22/layout_clearance_2.json','manufacturing_release':False}
(ROOT/'03_system/selected_electronics_v22.json').write_text(json.dumps(selected,indent=2)+'\n',encoding='utf-8')
segment=json.loads((ROOT/'04_kinematics/recovery_v22/segment_samples.json').read_text());prior=json.loads((ROOT/cfg['source_motion']).read_text());lookup={r['h_mm']:r for r in prior['samples']};results=[]

def inspect(p0,info,T,baseline,all_pairs=False):
    parts=apply_layout(p0,T,layout);volume={n:abs(parts[n].Volume-p0[n].Volume) for n in changed};assert max(volume.values())<1e-6
    failed,near=changed_audit(parts,changed,nominal)
    if all_pairs:collisions,_=changed_audit(parts,set(parts),0)
    else:collisions=[c for c in baseline['collisions'] if c['a'] not in changed and c['b'] not in changed]
    mass,com=mass_com(parts,info);before_mass,before_com=mass_com(p0,info);assert abs(mass-before_mass)<1e-9
    shift=V()
    for n,d in layout.items():shift+=T.Rotation.multVec(V(*d))*(info[n]['mass_kg']/mass)
    assert (V(*com)-V(*before_com)-shift).Length<1e-6
    bb=g.boxes(parts);ground={n:s.BoundBox.ZMin for n,s in parts.items() if bb[n].ZMin<-.5 and s.BoundBox.ZMin<-.5}
    return parts,{'clearance_failures':failed,'near_clearance_pairs':near,'other_or_full_collisions':collisions,'full_pair_audit':all_pairs,
        'ground_penetrations_mm':ground,'mass_kg':mass,'COM_mm':com,'COM_change_mm':list(shift),'volume_errors_mm3':volume,
        'geometry_clearance_pass':not failed and not collisions and not ground}

for sample in segment['samples']:
    h=sample['h_mm'];p,info,T=motion(sample);full=not sample['from_v21_exact_sample'] or h in (114,117,119)
    parts,r=inspect(p,info,T,lookup.get(h),full);r['h_mm']=h;r['support']=assess(segment['support_points_mm'],r['COM_mm'],r['mass_kg']);r['static_balance']=reactions(segment['support_points_mm'],r['COM_mm'],r['mass_kg'])
    r['sample_geometry_and_vertical_balance_pass']=r['geometry_clearance_pass'] and r['static_balance']['nonnegative_vertical_balance_exists']
    if h in (114,117,119):save('lift_h'+format(h,'g'),parts,info)
    results.append(r)
    (OUT/'motion_validation.json').write_text(json.dumps({'layout':layout,'samples':results,'complete':False},indent=2)+'\n',encoding='utf-8')
    print('validated',h,'geometry',r['geometry_clearance_pass'],'margin',round(r['support']['support_polygon_margin_mm'],3),'full pairs',full,flush=True)
p,info,T,oldstand=stand();parts,standing=inspect(p,info,T,oldstand,True);save('stand_candidate',parts,info)
standpoints=[[x,y,0] for x in (-50,50) for y in (-87.5,-32.5,32.5,87.5)];standing['static_balance']=reactions(standpoints,standing['COM_mm'],standing['mass_kg'])
bb=Part.makeCompound(list(parts.values())).BoundBox;standing['bounds_size_mm']=[bb.XLength,bb.YLength,bb.ZLength];standing['maximum_z_mm']=bb.ZMax;standing['source_bounds_size_mm']=oldstand['bounds']['size_mm']
out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'selected_layout':selected,'samples':results,'standing':standing,'complete':True,'full_recovery_validated':False,
     'scope':'13 half-mm samples plus standing. Full pairs at three integer endpoints and all six new midpoint poses; unchanged pairs at other exact frozen v21 integer poses reused from prior evidence. No continuous collision, structural strength, mounting/cabling or hardware validation.'}
(OUT/'motion_validation.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('standing',standing['geometry_clearance_pass'],'height',standing['maximum_z_mm'],'mass',standing['mass_kg'],flush=True)
