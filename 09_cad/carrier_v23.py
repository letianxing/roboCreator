"""Reference adjustable carrier, retaining straps, fastener envelopes and routed harness keepout."""
import json,math,sys
from pathlib import Path
import FreeCAD as A,Part
import numpy as np
import geometry_v22 as prior
ROOT=prior.ROOT;V=A.Vector;PF=ROOT/'03_system/electronics_carrier_v23.json';cfg=json.loads(PF.read_text());OUT=ROOT/'09_cad/body_v23';OUT.mkdir(exist_ok=True)
rho=cfg['densities_kg_mm3'];g=prior.g

def box(lo,hi):return Part.makeBox(*(b-a for a,b in zip(lo,hi)),V(*lo))
def unite(parts):
    s=parts[0]
    for p in parts[1:]:s=s.fuse(p)
    return s.removeSplitter()
def ring_box(lo,hi,ilo,ihi):return box(lo,hi).cut(box(ilo,ihi))
def band_yz(x,zlo,zhi,yhalf,width=8,t=1):
    return ring_box((x-width/2,-yhalf-t,zlo-t),(x+width/2,yhalf+t,zhi+t),(x-width/2-1,-yhalf,zlo),(x+width/2+1,yhalf,zhi))
def band_xy(z,width=8):
    return ring_box((-60,-38,z),(-44.5,38,z+width),(-59,-37,z-1),(-45.5,37,z+width+1))
def rounded_path(points,radius,tube_radius):
    points=[V(*p) for p in points];edges=[];current=points[0];arcs=[]
    for a,b,c in zip(points,points[1:],points[2:]):
        u=(b-a).normalize();v=(c-b).normalize();dot=max(-1,min(1,u.dot(v)));angle=math.acos(dot);d=radius*math.tan(angle/2)
        assert d<(b-a).Length and d<(c-b).Length
        t1=b-u*d;t2=b+v*d;normal=(v-u*dot).normalize();center=t1+normal*radius;mid=center+((t1-center)+(t2-center)).normalize()*radius
        if (t1-current).Length>1e-8:edges.append(Part.makeLine(current,t1))
        edges.append(Part.Arc(t1,mid,t2).toShape());arcs.append({'radius_mm':radius,'tangent_before_mm':list(t1),'tangent_after_mm':list(t2)});current=t2
    edges.append(Part.makeLine(current,points[-1]));wire=Part.Wire(edges);profile=Part.Wire([Part.makeCircle(tube_radius,points[0],(points[1]-points[0]).normalize())]);solid=wire.makePipeShell([profile],True,False)
    assert solid.isValid() and solid.Volume>0
    return solid,arcs,wire.Length

def native():
    parts={};meta={}
    def put(n,s,material,role='structure'):
        assert s.isValid() and s.Volume>0,n
        parts[n]=s;meta[n]={'group':'electronics_mount_v23','role':role,'source_type':'CAD_density_reference' if material else 'assumed_keepout','mass_kg':s.Volume*rho[material] if material else None,'material_assumption':material}
    floor=box((-45.4,-22.4,54),(39.4,22.4,56));wall=[box((-45.4,-22.4,56),(-43.4,22.4,64)),box((37.4,-22.4,56),(39.4,22.4,64)),box((-43.4,-22.4,56),(37.4,-20.4,64)),box((-43.4,20.4,56),(37.4,22.4,64))]
    spine=box((-47.5,-37,54),(-45.5,37,118.5)).cut(box((-48,-25,64),(-45,25,105)))
    tongue=box((-47.5,-22.4,54),(-43.5,22.4,56));gussets=[]
    for side in (-1,1):
        face=Part.Face(Part.makePolygon([V(-47.5,22.4,54),V(-32.5,22.4,54),V(-47.5,22.4,70),V(-47.5,22.4,54)]));s=face.extrude(V(0,2,0));gussets.append(s if side==1 else s.mirror(V(),V(0,1,0)))
    posts=[Part.makeCylinder(3.25,4,V(x,y,50)) for x,y in cfg['carrier_screw_centers_body_xy_mm']]
    carrier=unite([floor,spine,tongue]+wall+gussets+posts)
    for x,y in cfg['carrier_screw_centers_body_xy_mm']:
        carrier=carrier.cut(Part.makeCylinder(1.4,8,V(x,y,49)))
        carrier=carrier.cut(Part.makeCone(1.4,2.65,1.25,V(x,y,54.5)))
        carrier=carrier.cut(Part.makeCylinder(2.65,.5,V(x,y,55.75)))
    put('ElectronicsCarrier',carrier.removeSplitter(),'PETG_reference')
    put('BatteryPadding',box((-42,-19,56),(36,19,58)),'foam_assumed','pad')
    for i,x in enumerate((-28,28)):put('BatteryRetainer'+str(i),band_yz(x,54,90,22.4),'strap_and_grommet_assumed','retainer')
    for i,(y,z) in enumerate([(y,z) for y in (-28,28) for z in (80,108)]):put('ComputerCushion'+str(i),box((-49,y-3,z-4),(-47.5,y+3,z+4)),'foam_assumed','pad')
    for i,z in enumerate((74,104)):put('ComputerRetainer'+str(i),band_xy(z),'strap_and_grommet_assumed','retainer')
    put('PowerPadding',box((-38,-14,44),(4,14,46)),'foam_assumed','pad')
    for i,x in enumerate((-30,1)):put('PowerRetainer'+str(i),band_yz(x,32,50,17.5,width=6),'strap_and_grommet_assumed','retainer')
    for i,(x,y) in enumerate(cfg['carrier_screw_centers_body_xy_mm']):
        shank=Part.makeCylinder(1.25,8.5,V(x,y,46));head=Part.makeCone(1.25,2.5,1.25,V(x,y,54.5));cap=Part.makeCylinder(2.5,.25,V(x,y,55.75));put('CarrierScrew'+str(i),unite([shank,head,cap]),'steel_assumed','fastener')
    plate=box((-44,-50,46),(25,50,50))
    for x,y in cfg['carrier_screw_centers_body_xy_mm']:plate=plate.cut(Part.makeCylinder(1.3,6,V(x,y,45)))
    for x in (-30,1):
        for y in (-18,18):plate=plate.cut(box((x-3.4,y-1.2,45),(x+3.4,y+1.2,51)))
    plate=plate.cut(Part.makeCylinder(4.25,6,V(-36,-40,45)))
    put('PelvisPlate',plate.removeSplitter(),'aluminum_assumed')
    grommet=Part.makeCylinder(4.25,4,V(-36,-40,46)).fuse(Part.makeCylinder(5.25,1,V(-36,-40,50))).cut(Part.makeCylinder(3.3,7,V(-36,-40,45)))
    put('HarnessGrommet',grommet,'strap_and_grommet_assumed','grommet')
    harness,arcs,length=rounded_path(cfg['harness_probe']['path_body_mm'],8,2)
    connectors=[box((-43,-36,71),(-33,-20,81)),box((8,-5,32),(26,5,44))]
    put('PowerHarnessKeepout',unite([harness]+connectors),None,'keepout')
    return parts,meta,{'bend_arcs':arcs,'centerline_length_mm':length,'connector_locations_unconfirmed':True}

def assemble(base,info,T,parts,meta):
    out={n:s.copy() for n,s in base.items()};mi={n:dict(r) for n,r in info.items()}
    out['battery'].translate(T.Rotation.multVec(V(*cfg['battery_delta_from_v22_body_mm'])))
    for n,s in parts.items():out[n]=g.env['placed'](s,T);mi[n]=dict(meta[n])
    return out,mi

def save(name,parts,info):
    doc=A.newDocument(name);objects=[];mesh=[]
    for n,s in parts.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope');o.Scope='v23 reference carrier; interfaces,preload,CAE and manufacturing unvalidated';objects.append(o)
        if info[n]['role'] in ('motor','electronics'):
            b=s.BoundBox;vis=Part.makeBox(b.XLength,b.YLength,b.ZLength,V(b.XMin,b.YMin,b.ZMin))
        else:vis=s
        pts,tris=vis.tessellate(1);mesh.append({'name':n,'role':info[n]['role'],'points':[list(v) for v in pts],'triangles':tris})
    doc.recompute();path=OUT/(name+'.FCStd');doc.saveAs(str(path));Part.export(objects,str(OUT/(name+'.step')));A.closeDocument(doc.Name)
    doc=A.openDocument(str(path));assert all(o.Shape.isValid() for o in doc.Objects if hasattr(o,'Shape'));count=len(doc.Objects);A.closeDocument(doc.Name)
    (OUT/(name+'_mesh.json')).write_text(json.dumps(mesh)+'\n',encoding='utf-8');return count

if __name__=='__main__':
    p,m,h=native();count=save('carrier_components',p,m)
    (OUT/'carrier_native.json').write_text(json.dumps({'component_count':count,'components':m,'harness':h,'scope':'Reference envelopes, no mounting or manufacturing release'},indent=2)+'\n',encoding='utf-8')
    print('Native carrier valid parts',count,'known mass kg',sum(r['mass_kg'] or 0 for r in m.values()),flush=True)
