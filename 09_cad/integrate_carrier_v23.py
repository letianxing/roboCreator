"""Integrate retained v23 draft into frozen v22 poses and audit all changed pairs."""
import json,hashlib,argparse
import carrier_v23 as c
from geometry_v22 import motion,stand,apply_layout,mass_com,changed_audit,g,assess,reactions
ROOT=c.ROOT;V=c.V

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--all-samples',action='store_true');parser.add_argument('--revised',action='store_true');parser.add_argument('--fit',action='store_true');args=parser.parse_args()
    if args.fit:args.revised=True
    out=ROOT/('09_cad/body_v23_fit' if args.fit else '09_cad/body_v23_revision' if args.revised else '09_cad/body_v23_integration');out.mkdir(exist_ok=True);c.OUT=out
    if args.revised:
        import revise_carrier_v23 as revision
        native,meta,harness=revision.native(fit=args.fit)
    else:native,meta,harness=c.native()
    layout=json.loads((ROOT/'03_system/selected_electronics_v22.json').read_text())['translations_relative_v21_body_mm']
    source=ROOT/'04_kinematics/recovery_v22/segment_samples.json';segment=json.loads(source.read_text());oldfile=ROOT/'09_cad/body_v22/motion_validation.json';baseline=json.loads(oldfile.read_text())
    assert baseline['complete'] and all(r['geometry_clearance_pass'] for r in baseline['samples'])
    if args.fit:
        oldfile=ROOT/'09_cad/body_v23_revision/integration.json';previous=json.loads(oldfile.read_text())
        assert previous['completed'] and len(previous['scenes'])==14 and all(r['geometry_pass'] for r in previous['scenes'])
    cases=[s for s in segment['samples'] if args.all_samples or s['h_mm'] in (114,117,120)]
    results=[];new_names={'ElectronicsCarrier','HarnessGrommet','PelvisPlate'} if args.fit else set(native)|{'battery'}
    for sample in cases+[None]:
        if sample is None:
            p,info,T,_=stand();name='stand';support=[[x,y,0] for x in (-50,50) for y in (-87.5,-32.5,32.5,87.5)]
        else:
            p,info,T=motion(sample);name='lift_h'+format(sample['h_mm'],'g');support=segment['support_points_mm']
        p=apply_layout(p,T,layout);oldmass,_=mass_com(p,info);removed=info['PelvisPlate']['mass_kg']
        assembled,mi=c.assemble(p,info,T,native,meta);failed,near=changed_audit(assembled,new_names,0)
        mass,com=mass_com(assembled,mi);expected=oldmass-removed+sum(m['mass_kg'] or 0 for m in meta.values());assert abs(mass-expected)<1e-8
        bb=g.boxes(assembled);ground={n:s.BoundBox.ZMin for n,s in assembled.items() if bb[n].ZMin<-.5 and s.BoundBox.ZMin<-.5}
        row={'scene':name,'h_mm':sample['h_mm'] if sample else None,'changed_pair_failures':failed,'changed_near_pairs':near,
             'ground_penetrations_mm':ground,'component_count':len(assembled),'motor_count':sum(r['role']=='motor' for r in mi.values()),
             'old_mass_kg':oldmass,'removed_plate_mass_kg':removed,'added_native_mass_kg':sum(m['mass_kg'] or 0 for m in meta.values()),
             'new_mass_kg':mass,'COM_mm':com,'vertical_balance':reactions(support,com,mass),
             'geometry_pass':not failed and not ground,'manufacturing_release':False}
        if sample:row['support_margin']=assess(support,com,mass)
        if sample is None or sample['h_mm'] in (114,117,120):
            row['saved_valid_shapes']=c.save(name,assembled,mi)
        results.append(row)
        result={'completed':len(results)==len(cases)+1,'source_motion_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
          'source_geometry_evidence_sha256':hashlib.sha256(oldfile.read_bytes()).hexdigest(),'carrier_input_sha256':hashlib.sha256(c.PF.read_bytes()).hexdigest(),
          'revision_input_sha256':hashlib.sha256(revision.PF.read_bytes()).hexdigest() if args.revised else None,
          'fit_input_sha256':hashlib.sha256(revision.FIT.read_bytes()).hexdigest() if args.fit else None,
          'unchanged_pair_evidence':str(oldfile.relative_to(ROOT)),'changed_pair_names':sorted(new_names),
          'native_solid_counts':{n:len(s.Solids) for n,s in native.items()},'harness':harness,'scenes':results,
          'scope':'Modified pairs audited; unchanged pairs retain the referenced existing evidence. Reference microphone envelope retained; arrived UAC board is not dimensioned/included. No full recovery, material strength,preload or hardware approval.'}
        (out/'integration.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print(name,'pairs',len(failed),'ground',len(ground),'mass',round(mass,6),flush=True)
        print([(r['a'],r['b'],r.get('volume_mm3')) for r in failed],flush=True)
if __name__=='__main__':main()
