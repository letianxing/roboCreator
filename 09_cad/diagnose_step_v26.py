"""Locate STEP roundtrip faults and test bounded healing on independent copies."""
import json,hashlib
from pathlib import Path
import FreeCAD as A
import Part
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/step_repair_v26';OUT.mkdir(exist_ok=True)
SRC=ROOT/'09_cad/uac_mount_v25_service'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def info(s):
    # Display triangulation can alter BoundBox; compare unmeshed geometry copies.
    b=s.cleaned().BoundBox
    return {'valid':s.isValid(),'volume_mm3':s.Volume,'area_mm2':s.Area,'center_mm':list(s.CenterOfMass),
            'bbox_mm':[b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax],
            'faces':len(s.Faces),'edges':len(s.Edges),'vertices':len(s.Vertexes),'solids':len(s.Solids),
            'max_topology_tolerance_mm':s.getTolerance(1)}
def check(s,bop=False):
    try:return {'return':str(s.check(bop)),'error':None}
    except Exception as e:return {'error':str(e),'exception':type(e).__name__}
def delta(a,b):
    x,y=info(a),info(b)
    return {'volume_delta_mm3':y['volume_mm3']-x['volume_mm3'],
            'area_delta_mm2':y['area_mm2']-x['area_mm2'],
            'center_displacement_mm':(a.CenterOfMass-b.CenterOfMass).Length,
            'max_bbox_delta_mm':max(abs(i-j) for i,j in zip(x['bbox_mm'],y['bbox_mm'])),
            'faces_delta':len(b.Faces)-len(a.Faces)}

def main():
    step=Part.read(str(SRC/'stand.step'));doc=A.openDocument(str(SRC/'stand.FCStd'));rows=[]
    for idx,s in enumerate(step.Solids):
        if s.isValid():continue
        row={'STEP_solid_index':idx,'STEP':info(s),'check':check(s),'matches':[],'trials':[]}
        for o in doc.Objects:
            for si,n in enumerate(o.Shape.Solids):
                if (s.CenterOfMass-n.CenterOfMass).Length<.01 and abs(s.Volume-n.Volume)<max(.01,abs(n.Volume)*1e-4):
                    row['matches'].append({'object':o.Name,'native_solid_index':si,'native':info(n),'native_to_STEP':delta(n,s)})
        s.exportBrep(str(OUT/('invalid_'+str(idx)+'.brep')))
        for cap in (1e-7,1e-6,1e-5):
            fixed=s.copy();ret=fixed.fix(1e-7,1e-7,cap)
            trial={'cap_mm':cap,'fix_return':ret,'fixed':info(fixed),'delta':delta(s,fixed),'check':check(fixed)}
            file=OUT/('trial_'+str(idx)+'_'+str(cap)+'.step');fixed.exportStep(str(file));again=Part.read(str(file))
            trial['roundtrip_valid']=again.isValid();trial['roundtrip_solid_count']=len(again.Solids)
            if len(again.Solids)==1:trial['roundtrip_delta']=delta(s,again.Solids[0])
            row['trials'].append(trial)
        rows.append(row);print(json.dumps(row,indent=2),flush=True)
        (OUT/'diagnosis.json').write_text(json.dumps({'complete':False,'rows':rows},indent=2)+'\n')
    A.closeDocument(doc.Name)
    (OUT/'diagnosis.json').write_text(json.dumps({'complete':True,'FreeCAD_version':A.Version(),'OCC_version':Part.OCC_VERSION,
        'source_sha256':{n:sha(SRC/n) for n in ['stand.FCStd','stand.step']},'rows':rows},indent=2)+'\n')

if __name__=='__main__':main()
