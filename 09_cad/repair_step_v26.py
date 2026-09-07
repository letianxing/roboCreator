"""Narrow postprocessor for v25; preserve labels and all unrelated native shapes."""
import json,hashlib,math
from pathlib import Path
from rebuild_rollrod_v26 import rebuild,dimensions
from diagnose_step_v26 import A,Part,ROOT,OUT,info,delta,sha,check
from uac_mount_v25 import pair_audit,volcenter
PF=ROOT/'03_system/step_repair_v26.json'
cfg=json.loads(PF.read_text());LIMIT=cfg['numerical_acceptance']
DEST=OUT/'repaired';DEST.mkdir(exist_ok=True)

def brep_hash(s):return hashlib.sha256(s.exportBrepToString().encode()).hexdigest()
def serialization_compare(a,b):
    left=a.exportBrepToString().split();right=b.exportBrepToString().split()
    assert len(left)==len(right),'BRep token structure changed'
    diffs=[]
    for x,y in zip(left,right):
        if x==y:continue
        try:u,v=float(x),float(y)
        except ValueError:raise AssertionError('Non-numeric BRep token changed: '+x+' / '+y)
        assert math.isclose(u,v,rel_tol=1e-12,abs_tol=1e-12),(x,y)
        diffs.append(abs(u-v))
    return {'different_numeric_tokens':len(diffs),'max_numeric_token_delta':max(diffs,default=0)}
def sampled_boundary(a,b):
    results=[]
    for source,target in [(a,b),(b,a)]:
        points,_=source.copy().tessellate(LIMIT['surface_tessellation_deflection_mm'])
        boundary=Part.makeCompound(target.Faces)
        distances=[Part.Vertex(p).distToShape(boundary)[0] for p in points]
        results.append({'sample_count':len(points),'max_sample_distance_mm':max(distances)})
    return results
def accepted(change):
    return abs(change['volume_delta_mm3'])<=LIMIT['max_volume_delta_mm3'] and change['max_bbox_delta_mm']<=LIMIT['max_bbox_delta_mm'] and change['center_displacement_mm']<=LIMIT['max_center_delta_mm']

def main():
    source=ROOT/cfg['source_directory'];rows=[]
    base=json.loads((source/'integration.json').read_text())
    for name in cfg['scenes']:
        path=source/(name+'.FCStd');source_hash=sha(path);step_hash=sha(path.with_suffix('.step'))
        doc=A.openDocument(str(path));original={o.Name:o.Shape.copy() for o in doc.Objects}
        signatures={n:brep_hash(s) for n,s in original.items()};targets=[]
        for target in cfg['target_objects']:
            old=original[target];new=rebuild(old,cfg['eye_phase_deg'],cfg['stem_phase_deg'])
            assert new.isValid() and len(new.Solids)==1,target
            diff=delta(old.Solids[0],new.Solids[0]);assert accepted(diff),(name,target,diff)
            assert new.getTolerance(1)<=LIMIT['max_topology_tolerance_mm']
            samples=sampled_boundary(old,new)
            assert all(r['max_sample_distance_mm']<=LIMIT['max_boundary_sample_distance_mm'] for r in samples)
            obj=doc.getObject(target);obj.Shape=new
            obj.addProperty('App::PropertyString','RepairScope');obj.RepairScope='Same rod dimensions; solid union before one-time eye drilling. Not strength/manufacturing approval.'
            dims=dimensions(new)
            targets.append({'object':target,'before':info(old.Solids[0]),'after':info(new.Solids[0]),'difference':diff,
                'boundary_samples':samples,'eye_centers_local_mm':[list(e['point']) for e in dims],
                'eye_widths_mm':[e['width'] for e in dims],'native_check':check(new),
                'mass_delta_kg_at_inherited_steel_density':diff['volume_delta_mm3']*7.85e-6})
        doc.recompute();newparts={o.Name:o.Shape.copy() for o in doc.Objects}
        unchanged=[n for n in newparts if n not in cfg['target_objects']]
        assert all(brep_hash(newparts[n])==signatures[n] for n in unchanged),'Unrelated in-memory native shape changed'
        collisions=pair_audit(newparts,set(cfg['target_objects']));assert collisions['pass'],collisions['failed_pairs']
        outpath=DEST/(name+'.FCStd');doc.saveAs(str(outpath));Part.export(doc.Objects,str(outpath.with_suffix('.step')));A.closeDocument(doc.Name)
        d=A.openDocument(str(outpath));reloaded={o.Name:o.Shape.copy() for o in d.Objects};A.closeDocument(d.Name)
        assert list(reloaded)==list(original) and all(s.isValid() for s in reloaded.values())
        unchanged=[n for n in reloaded if n not in cfg['target_objects']]
        normalization={n:serialization_compare(original[n],reloaded[n]) for n in unchanged if brep_hash(reloaded[n])!=signatures[n]}
        imported=Part.read(str(outpath.with_suffix('.step')))
        expected_solids=sum(len(s.Solids) for s in reloaded.values())
        valid=imported.isValid();bad=[i for i,s in enumerate(imported.Solids) if not s.isValid()]
        assert valid and not bad and len(imported.Solids)==expected_solids,(name,valid,bad,len(imported.Solids),expected_solids)
        # Identify reimported rods by volume and COM, not export order.
        for target in targets:
            old=original[target['object']].Solids[0]
            matches=[(i,s) for i,s in enumerate(imported.Solids) if (s.CenterOfMass-old.CenterOfMass).Length<.001 and abs(s.Volume-old.Volume)<.01]
            assert len(matches)==1
            i,s=matches[0];target['STEP_solid_index']=i;target['STEP_vs_original']=delta(old,s)
            assert accepted(target['STEP_vs_original'])
            target['STEP_boundary_samples']=sampled_boundary(old,s)
            assert all(r['max_sample_distance_mm']<=LIMIT['max_boundary_sample_distance_mm'] for r in target['STEP_boundary_samples'])
            target['STEP_check']=check(s);assert target['STEP_check']['error'] is None
        bb=Part.makeCompound(list(reloaded.values())).cleaned().BoundBox
        oldbb=Part.makeCompound(list(original.values())).cleaned().BoundBox
        bbdelta=max(abs(getattr(bb,k)-getattr(oldbb,k)) for k in ('XMin','XMax','YMin','YMax','ZMin','ZMax'));assert bbdelta<.001
        assert sha(path)==source_hash and sha(path.with_suffix('.step'))==step_hash
        # Only microscopic rod-volume deltas, explicitly recorded rather than rounding away.
        massdelta=sum(t['mass_delta_kg_at_inherited_steel_density'] for t in targets)
        row={'scene':name,'source_native_sha256':source_hash,'source_STEP_sha256':step_hash,
             'targets':targets,'unchanged_in_memory_shape_hash_count':len(unchanged),'reopened_unrelated_numeric_normalization':normalization,'component_count':len(reloaded),
             'reimported_STEP_valid':valid,'reimported_STEP_solid_count':len(imported.Solids),
             'source_native_solid_count':sum(len(s.Solids) for s in original.values()),
             'changed_pair_audit':collisions,'assembly_bbox_max_delta_mm':bbdelta,
             'CAD_mass_delta_kg':massdelta,'manufacturing_release':False,
             'output_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [outpath,outpath.with_suffix('.step')]}}
        rows.append(row)
        (OUT/'repair_results.json').write_text(json.dumps({'complete':False,'scenes':rows},indent=2)+'\n')
        print('REPAIRED',name,'STEP solids',len(imported.Solids),'unchanged objects',len(unchanged),flush=True)
    result={'complete':True,'input_sha256':sha(PF),'FreeCAD_version':A.Version(),'OCC_version':Part.OCC_VERSION,
        'scenes':rows,'method':'Rebuild source-owned two-eye cylinder rods; solid union first, each hole drilled once; same Placement and dimensions',
        'scope':'Native and STEP exchange validation only. No actual hardware, full dynamics/CAE or manufacturing release.',
        'global_import_export_preferences_changed':False,'source_files_changed':False}
    (OUT/'repair_results.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
