"""Rebuild the same cylindrical rod/eyes, drilling only after solid union."""
import json
from diagnose_step_v26 import A,Part,ROOT,OUT,info,delta,check
V=A.Vector

def dimensions(source):
    local=source.copy();local.Placement=A.Placement();eyes=[]
    for f in local.Faces:
        surf=f.Surface
        if type(surf).__name__!='Cylinder' or abs(surf.Radius-5)>1e-6:continue
        axis=surf.Axis;origin=surf.Center
        distances=[(v.Point-origin).dot(axis) for v in f.Vertexes]
        middle=origin+axis*((min(distances)+max(distances))/2)
        # Seam changes may split one analytic cylinder into multiple faces.
        if not any((e['point']-middle).Length<1e-6 and abs(abs(e['axis'].dot(axis))-1)<1e-6 for e in eyes):
            eyes.append({'point':middle,'axis':axis,'width':max(distances)-min(distances)})
    assert len(eyes)==2
    assert abs((eyes[1]['point']-eyes[0]['point']).Length-28)<1e-6
    assert all(abs(e['width']-4)<1e-6 for e in eyes)
    assert abs(abs(eyes[0]['axis'].dot(eyes[1]['axis']))-1)<1e-6
    return eyes

def rebuild(source,eye_phase=0,stem_phase=0,stem_inset=0):
    eyes=dimensions(source);a,b=[e['point'] for e in eyes];axis=eyes[0]['axis'];direction=(b-a).normalize()
    assert 0<=stem_inset<=(25-4)**.5
    # A stem portion up to sqrt(R_eye^2-R_stem^2) is wholly covered by an eye.
    # Removing only this buried construction overlap does not shorten the finished link.
    stem=Part.makeCylinder(2,(b-a).Length-2*stem_inset,a+direction*stem_inset,direction);stem.rotate(a,direction,stem_phase)
    allparts=[stem]
    for point in (a,b):
        eye=Part.makeCylinder(5,4,point-axis*2,axis);eye.rotate(point,axis,eye_phase);allparts.append(eye)
    shape=allparts[0].fuse(allparts[1]).fuse(allparts[2]).removeSplitter()
    for point in (a,b):shape=shape.cut(Part.makeCylinder(2,20,point-axis*10,axis))
    shape=shape.removeSplitter();shape.Placement=source.Placement
    return shape

def main():
    doc=A.openDocument(str(ROOT/'09_cad/uac_mount_v25_service/stand.FCStd'));source=doc.getObject('L_RollRod').Shape.copy();A.closeDocument(doc.Name)
    results=[]
    for ep,sp in [(0,0),(45,0),(90,0),(0,45),(45,45),(90,45)]:
        candidate=rebuild(source,ep,sp);path=OUT/('rebuild_eye'+str(ep)+'_stem'+str(sp)+'.step');candidate.exportStep(str(path))
        again=Part.read(str(path));row={'eye_phase_deg':ep,'stem_phase_deg':sp,'native':info(candidate.Solids[0]),
             'native_delta':delta(source.Solids[0],candidate.Solids[0]),'roundtrip_valid':again.isValid(),
             'roundtrip_check':check(again),'roundtrip_solids':len(again.Solids)}
        if len(again.Solids)==1:row['roundtrip_delta']=delta(source.Solids[0],again.Solids[0])
        results.append(row);print(ep,sp,row,flush=True)
    (OUT/'rebuild_trials.json').write_text(json.dumps({'complete':True,'trials':results},indent=2)+'\n')

if __name__=='__main__':main()
