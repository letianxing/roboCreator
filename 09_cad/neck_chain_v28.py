"""Explicit body -> yaw carriage -> pitched head chain, preserving failure evidence."""
import json,itertools,math
from pathlib import Path
import FreeCAD as A
import Part
from uac_space_v24 import sha,gap
ROOT=Path(__file__).resolve().parents[1];PF=ROOT/'03_system/neck_v28.json';cfg=json.loads(PF.read_text());OUT=ROOT/'09_cad/neck_v28';OUT.mkdir(exist_ok=True)
V=A.Vector
FIXED_BOUNDS={}
INTERNAL_PAIRS={}
def placed(s,T):
    t=s.copy();t.Placement=T.multiply(s.Placement);return t
def around(point,axis,degrees):
    R=A.Rotation(V(*axis),degrees);p=V(*point);return A.Placement(p-R.multVec(p),R)
def load():
    doc=A.openDocument(str(ROOT/cfg['source_CAD']));parts={o.Name:o.Shape.copy() for o in doc.Objects};A.closeDocument(doc.Name)
    T=A.Placement(V(*cfg['body_origin_world_mm']),A.Rotation());local={n:placed(s,T.inverse()) for n,s in parts.items()}
    local['camera'].translate(V(15,0,0))
    return parts,local,T
def build(local,T,pivot,yaw,pitch):
    Y=around(cfg['yaw_origin_body_mm'],cfg['yaw_axis_parent'],yaw)
    P=around(pivot,cfg['pitch_axis_parent'],pitch);H=Y.multiply(P)
    parts=dict(local)
    for name in cfg['head_rigid_group']:parts[name]=placed(local[name],H)
    m=local['NeckMotor1'].copy();c=m.Solids[0].CenterOfMass;target=V(*pivot)+V(*cfg['pitch_motor_center_offset_mm']);m.translate(target-c);parts['NeckMotor1']=placed(m,Y)
    return {n:placed(s,T) for n,s in parts.items()},T.multiply(H),T.multiply(Y)
def audit(parts,changed):
    bad=[];near=[];boxes={}
    for n,s in parts.items():
        if n not in changed:
            if n not in FIXED_BOUNDS:FIXED_BOUNDS[n]=s.cleaned().BoundBox
            boxes[n]=FIXED_BOUNDS[n]
        else:boxes[n]=s.cleaned().BoundBox
    for a,b in itertools.combinations(parts,2):
        if not ({a,b}&changed):continue
        if gap(boxes[a],boxes[b])>1e-5:continue
        internal=a in cfg['head_rigid_group'] and b in cfg['head_rigid_group']
        key=tuple(sorted((a,b)))
        if internal and key in INTERNAL_PAIRS:volume=INTERNAL_PAIRS[key]
        else:
            distance=parts[a].distToShape(parts[b])[0]
            volume=parts[a].common(parts[b]).Volume if distance<1e-7 else 0
            if internal:INTERNAL_PAIRS[key]=volume
        if volume>1e-5:bad.append({'a':a,'b':b,'volume_mm3':volume})
    return bad
def save(name,parts):
    doc=A.newDocument(name)
    for n,s in parts.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope');o.Scope='v28 two-axis kinematic proposal; hardware brackets/cables/actuator and dynamic validation incomplete'
    doc.recompute();path=OUT/(name+'.FCStd');doc.saveAs(str(path));A.closeDocument(doc.Name)
    d=A.openDocument(str(path));assert len(d.Objects)==118 and all(o.Shape.isValid() for o in d.Objects);A.closeDocument(d.Name)
def main():
    old,local,T=load();changed=set(cfg['head_rigid_group']+cfg['yaw_carriage_group']);rows=[]
    for idx,pivot in enumerate(cfg['pitch_origin_candidates_body_mm']):
        cases=[]
        for pitch in cfg['pitch_scan_deg']:
            parts,H,Y=build(local,T,pivot,0,pitch);bad=audit(parts,changed)
            cases.append({'yaw_deg':0,'pitch_deg':pitch,'collisions':bad,'pass':not bad})
            print('sample',idx,pitch,'collisions',len(bad),flush=True)
        row={'id':idx,'pitch_origin_body_mm':pivot,'cases':cases,'pitch_pass_count':sum(c['pass'] for c in cases)};rows.append(row)
        (OUT/'scan.json').write_text(json.dumps({'complete':False,'candidates':rows},indent=2)+'\n')
        print(idx,pivot,'pass',row['pitch_pass_count'],'/',len(cases),flush=True)
    # Keep best tested family, never interpret a best failure as approval.
    best=max(rows,key=lambda r:(r['pitch_pass_count'],-sum(len(c['collisions']) for c in r['cases']),-r['id']))
    yawrows=[]
    for yaw,pitch in itertools.product(cfg['yaw_probe_deg'],[0,30,45,60]):
        parts,H,Y=build(local,T,best['pitch_origin_body_mm'],yaw,pitch);bad=audit(parts,changed)
        yawrows.append({'yaw_deg':yaw,'pitch_deg':pitch,'collisions':bad,'pass':not bad})
        if (yaw,pitch) in [(0,0),(0,45),(0,60),(30,45)]:save('candidate_y'+str(yaw)+'_p'+str(pitch),parts)
    result={'complete':True,'input_sha256':sha(PF),'source_CAD_sha256':sha(ROOT/cfg['source_CAD']),'candidates':rows,'selected_for_further_study':best['id'],'yaw_pitch_cases':yawrows,
        'scope':'Two proposed joint frames, rigid head/crown/camera and yaw-mounted pitch housing. No actual horn/shaft/brackets, cable sweep, continuous collision proof or hardware approval.'}
    (OUT/'scan.json').write_text(json.dumps(result,indent=2)+'\n')
    print('selected',best['id'],'yaw pass',sum(r['pass'] for r in yawrows),flush=True)
if __name__=='__main__':main()
