"""Separate loose NURBS control-hull boxes from actual crown-height witnesses."""
import json,math
from pathlib import Path
import FreeCAD as A
import Part
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/neck_hardware_v28_final'
def refined_upper(face):
    original=face.Surface
    if type(original).__name__!='BSplineSurface':return {'kind':type(original).__name__,'upper_Z_mm':face.cleaned().BoundBox.ZMax}
    s=original.copy()
    for _ in range(5):
        u=s.getUKnots();v=s.getVKnots()
        for a,b in zip(u,u[1:]):s.insertUKnot((a+b)/2,1,1e-12)
        for a,b in zip(v,v[1:]):s.insertVKnot((a+b)/2,1,1e-12)
    assert min(w for row in s.getWeights() for w in row)>0
    bound=max(p.z for row in s.getPoles() for p in row)
    u=original.getUKnots();v=original.getVKnots();err=[]
    for a in [.1,.3,.5,.7,.9]:
        x=u[0]+(u[-1]-u[0])*a;y=v[0]+(v[-1]-v[0])*a
        err.append((s.value(x,y)-original.value(x,y)).Length)
    assert max(err)<1e-8
    return {'kind':'positive-weight rational BSpline control-hull after knot insertion','upper_Z_mm':bound,'geometry_sample_error_mm':max(err)}
def main():
    doc=A.openDocument(str(OUT/'compact_cable_y0_p50.FCStd'));shape=doc.getObject('WukongHeadShell').Shape.copy();A.closeDocument(doc.Name)
    bounds=[refined_upper(f) for f in shape.Faces]
    doc=A.openDocument(str(OUT/'y0_p0.FCStd'));crown=doc.getObject('CrownOccupancy').Shape;records=[]
    h=214.26539554219744;px,pz=5,146
    for vertex in crown.Vertexes:
        p=vertex.Point-A.Vector(0,0,h);a=-(p.x-px);b=p.z-pz;angles=[0,50]
        candidate=math.degrees(math.atan2(a,b))
        if 0<=candidate<=50:angles.append(candidate)
        for t in angles:records.append({'pitch_deg':t,'world_Z_mm':h+pz+a*math.sin(math.radians(t))+b*math.cos(math.radians(t)),'neutral_vertex_body_mm':list(p)})
    A.closeDocument(doc.Name);maximum=max(records,key=lambda r:r['world_Z_mm'])
    result={'head_at_pitch50_raw_bound_Z_mm':shape.cleaned().BoundBox.ZMax,'head_at_pitch50_refined_surface_upper_Z_mm':max(r['upper_Z_mm'] for r in bounds),
            'surface_bounds':bounds,'crown_exact_max_over_pitch0_to50':maximum,
            'scope':'Refinement is on geometry copies only. Positive NURBS weights give a conservative full-surface bound including trimmed-away portions. Crown is the existing rectangular occupancy, not final costume.'}
    (OUT/'height_refined.json').write_text(json.dumps(result,indent=2)+'\n');print(result['head_at_pitch50_refined_surface_upper_Z_mm'],maximum)
if __name__=='__main__':main()
