"""Read-only audit of all v16 shape pairs, including former exclusions."""
import FreeCAD as A, Part, json, hashlib, itertools
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'11_verification/v16_interface_audit_v17.json'
records=[]
for file in ['09_cad/support_mechanisms_v16/ankle_p0_p0.FCStd','09_cad/support_mechanisms_v16/hand_p0_p0.FCStd']:
    path=ROOT/file;before=hashlib.sha256(path.read_bytes()).hexdigest();doc=A.openDocument(str(path))
    parts={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};rows=[]
    for a,b in itertools.combinations(parts,2):
        sa,sb=parts[a],parts[b];volume=sa.common(sb).Volume
        distance=sa.distToShape(sb)[0]
        rows.append({'a':a,'b':b,'common_volume_mm3':volume,'distance_mm':distance})
    record={'source':file,'source_sha256':before,'pairs':rows,'multi_solid_parts':{k:len(s.Solids) for k,s in parts.items() if len(s.Solids)>1}}
    if 'ankle' in file:
        record['axis_support_gaps_mm']={k:parts[k].distToShape(parts['FixedBearings'])[0] for k in ['FixedFrame','RollMotorVendor','RollCarrier']}
    records.append(record);A.closeDocument(doc.Name)
    assert hashlib.sha256(path.read_bytes()).hexdigest()==before
out={'status':'audit_findings_not_assembly_pass','records':records,'interpretation':'Finite overlap must be evaluated by material and rigid body; excluded pair labels are not evidence of a valid mechanical interface. Disjoint frame solids require specified connecting parts.'}
OUT.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
for r in records:
    print(r['source'], 'positive overlaps',[(p['a'],p['b'],round(p['common_volume_mm3'],2)) for p in r['pairs'] if p['common_volume_mm3']>1e-5], 'multi solids',r['multi_solid_parts'])
    if 'axis_support_gaps_mm' in r:print(r['axis_support_gaps_mm'])
