"""Incremental exact collision audit after rigid electronics relocation."""
import FreeCAD as A, Part, json, hashlib, itertools
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v18';PF=ROOT/'03_system/electronics_repack_v18.json';p=json.loads(PF.read_text())
data=json.loads((OUT/'body_results.json').read_text());old=next(s for s in data['scenes'] if s['name']==p['source_scene'])
source=OUT/(p['source_scene']+'.FCStd');before=hashlib.sha256(source.read_bytes()).hexdigest();doc=A.openDocument(str(source))
parts={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name);V=A.Vector
changed={r['component'] for r in p['changes']};volume_checks=[];com_delta=V(0,0,0)
for row in p['changes']:
    s=parts[row['component']];oldvolume=s.Volume;c=s.CenterOfMass
    if row['component']=='PelvisPlate':assert abs(row['mass_kg']-oldvolume*2.7e-6)<1e-10
    if row.get('rotation_world_z_deg'):s.rotate(c,V(0,0,1),row['rotation_world_z_deg'])
    s.translate(V(*row['translation_world_mm']));assert s.isValid() and abs(s.Volume-oldvolume)<1e-6
    com_delta+=(s.CenterOfMass-c)*row['mass_kg'];volume_checks.append({'component':row['component'],'old_volume_mm3':oldvolume,'new_volume_mm3':s.Volume})
def group(name):
    if name in ['battery','microphone','computer','camera','power']:return 'electronics'
    if name.startswith('Neck'):return 'neck'
    if name=='PelvisPlate':return 'body'
    for c in old['collisions']:
        if c['a']==name:return c['group_a']
        if c['b']==name:return c['group_b']
    return 'other'
collisions=[c for c in old['collisions'] if c['a'] not in changed and c['b'] not in changed]
checked=0
for a,b in itertools.combinations(parts,2):
    if a not in changed and b not in changed:continue
    checked+=1;sa,sb=parts[a],parts[b]
    if sa.BoundBox.intersect(sb.BoundBox):
        v=sa.common(sb).Volume
        if v>1e-5:collisions.append({'a':a,'b':b,'volume_mm3':v,'group_a':group(a),'group_b':group(b)})
new=A.newDocument('stand_electronics_repack');objects=[];mesh=[]
for n,s in parts.items():
    o=new.addObject('PartDesign::Feature',n);o.Shape=s;objects.append(o)
    role='electronics' if n in ['battery','microphone','computer','power','camera'] else 'motor' if 'Motor' in n else 'shell' if n in ['WukongHeadShell','MonkeyEarm1','MonkeyEar1','CrownOccupancy'] else 'structure'
    bb=s.BoundBox;visual=Part.makeBox(bb.XLength,bb.YLength,bb.ZLength,V(bb.XMin,bb.YMin,bb.ZMin)) if role in ('motor','electronics') else s
    pts,tris=visual.tessellate(1);mesh.append({'name':n,'role':role,'group':group(n),'points':[list(v) for v in pts],'triangles':tris})
new.recompute();path=OUT/'stand_electronics_repack.FCStd';new.saveAs(str(path));Part.export(objects,str(OUT/'stand_electronics_repack.step'));A.closeDocument(new.Name)
reopened=A.openDocument(str(path));assert all(o.Shape.isValid() for o in reopened.Objects if hasattr(o,'Shape'));A.closeDocument(reopened.Name)
bb=Part.makeCompound(list(parts.values())).BoundBox
out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'source_sha256':before,'name':'stand_electronics_repack','changed_components':sorted(changed),
 'volume_checks':volume_checks,'incremental_exact_pairs_checked':checked,'unchanged_pairs_reused_from':'body_results.json/stand_narrow',
 'old_collision_count':len(old['collisions']),'collisions':collisions,'width_mm':bb.YLength,'depth_mm':bb.XLength,
 'known_or_allocated_mass_kg':old['known_or_allocated_mass_kg'],'approx_COM_mm':list(V(*old['approx_COM_mm'])+com_delta/old['known_or_allocated_mass_kg']),
 'manufacturable':False,'task_verified':False,'limitations':['New mounting hardware not included in mass','Unpitched static layout only, motion must be revalidated','Remaining shoulder and arm/leg collisions retained']}
assert hashlib.sha256(source.read_bytes()).hexdigest()==before
(OUT/'electronics_repack_results.json').write_text(json.dumps(out,indent=2)+'\n');(OUT/'stand_electronics_repack_mesh.json').write_text(json.dumps(mesh)+'\n')
print('collisions',len(old['collisions']),'->',len(collisions),'remaining electronics pairs',[(c['a'],c['b']) for c in collisions if 'electronics' in (c['group_a'],c['group_b'])],flush=True)
