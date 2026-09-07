"""Owned removable cassette + carrier revision, with explicitly provisional interfaces."""
import itertools,json,math,argparse
from pathlib import Path
from uac_space_v24 import A,Part,V,ROOT,placed,gap,sha,margin

PF=ROOT/'03_system/uac_mount_v25.json'
cfg=json.loads(PF.read_text())
OUT=ROOT/'09_cad/uac_mount_v25'

def box(lo,hi):return Part.makeBox(*[b-a for a,b in zip(lo,hi)],V(*lo))
def fuse(shapes):
    s=shapes[0]
    for other in shapes[1:]:s=s.fuse(other)
    return s.removeSplitter()
def cylx(radius,length,x,y,z):return Part.makeCylinder(radius,length,V(x,y,z),V(1,0,0))
def volcenter(shape):
    solids=shape.Solids;assert solids and sum(s.Volume for s in solids)>0
    center=V()
    for solid in solids:center+=solid.CenterOfMass*solid.Volume
    return center/sum(s.Volume for s in solids)

def rounded(points,radius,tube):
    points=[V(*p) for p in points];trim=[]
    for a,b,c in zip(points,points[1:],points[2:]):
        u=(b-a).normalize();v=(c-b).normalize();angle=math.acos(max(-1,min(1,u.dot(v))))
        d=radius*math.tan(angle/2);normal=(v-u*u.dot(v)).normalize()
        start=b-u*d;end=b+v*d;center=start+normal*radius
        mid=center+((start-center)+(end-center)).normalize()*radius
        trim.append((d,start,end,Part.Arc(start,mid,end).toShape()))
    for i,(a,b) in enumerate(zip(points,points[1:])):
        used=(trim[i-1][0] if i else 0)+(trim[i][0] if i<len(trim) else 0)
        assert used<(b-a).Length-1e-6,'Adjacent fillets overlap'
    edges=[];current=points[0]
    for _,start,end,arc in trim:
        edges += [Part.makeLine(current,start),arc];current=end
    edges.append(Part.makeLine(current,points[-1]))
    wire=Part.Wire(edges);profile=Part.Wire([Part.makeCircle(tube,points[0],(points[1]-points[0]).normalize())])
    shape=wire.makePipeShell([profile],True,False)
    assert shape.isValid() and len(shape.Solids)==1
    return shape,{'centerline_length_mm':wire.Length,'bend_count':len(trim),'centerline_radius_mm':radius,'tube_radius_mm':tube,'actual_cable_spec_verified':False}

def native(carrier,seat_relief=False,port_relief=False,tool_relief=False):
    parts={};meta={}
    def put(name,shape,material):
        assert shape.isValid() and len(shape.Solids)==1,name
        parts[name]=shape;meta[name]={'material_assumption':material,'mass_kg':shape.Volume*cfg['densities_kg_mm3'][material] if material else None,'volume_mm3':shape.Volume}
    additions=[box((39,-41,62),(42,41,65))]
    for y in cfg['mount_screw_y_mm']:
        additions.append(box((38,y-4,62),(42,y+4,107)))
    # Ear attaches to existing back-spine material, not the head shell.
    additions.append(box((-47.5,35,100),(-43.5,63,102)))
    carrier=fuse([carrier]+additions)
    panel=box((42,-43,75),(44,43,107))
    panel=fuse([panel,box((44,-43,75),(56,43,77)),box((44,-43,105),(56,43,107))])
    for j,(y,z) in enumerate(itertools.product(cfg['mount_screw_y_mm'],cfg['mount_screw_z_mm'])):
        carrier=carrier.cut(cylx(1.4,6,37,y,z)).cut(box((37.9,y-2.6,z-2.6),(40.05,y+2.6,z+2.6)))
        panel=panel.cut(cylx(1.4,4,41,y,z))
        if seat_relief:panel=panel.cut(cylx(2.5,13 if tool_relief else 3,44,y,z))
        nut=box((38,y-2.4,z-2.4),(40,y+2.4,z+2.4)).cut(cylx(1.25,4,37,y,z))
        screw=fuse([cylx(1.25,6,38,y,z),cylx(2.25,2,44,y,z)])
        put('UACNut'+str(j),nut,'steel_assumed');put('UACScrew'+str(j),screw,'steel_assumed')
    for y in (49.6,60.4):carrier=carrier.cut(box((-48.3,y-.7,99),(-43.7,y+.7,103)))
    put('ElectronicsCarrier',carrier.removeSplitter(),'PETG_reference')
    if port_relief:
        for lo,hi in [((41.5,-60.5,82.5),(56.5,-34.5,99.5)),((41.5,34.5,82.5),(56.5,47.5,99.5))]:
            panel=panel.cut(box(lo,hi))
    put('UACCassette',panel.removeSplitter(),'PETG_reference')
    # No PCB holes: removable insulating adapter face remains blank.
    put('InsulationSheet_PROVISIONAL',box((44,-35,77),(44.5,35,105)),'soft_liner_assumed')
    collar=cylx(3,4,-48,55,105).cut(cylx(2,6,-49,55,105))
    put('ArrayLinerLower',collar.common(box((-49,51,101),(-43,59,105))),'soft_liner_assumed')
    put('ArrayLinerUpper',collar.common(box((-49,51,105),(-43,59,109))),'soft_liner_assumed')
    strap=box((-48,49.2,99.2),(-44,60.8,108.8)).cut(box((-49,50,100),(-43,60,108)))
    put('ArrayTie_PROVISIONAL',strap,'strap_assumed')
    cable,path=rounded(cfg['array_route_body_mm'],cfg['probe_centerline_radius_mm'],cfg['probe_bundle_radius_mm'])
    put('ArrayCable_PROBE',cable,None)
    put('RemainingBoardSpace_PROBE',box((44.5,-35,77),(56,35,105)),None)
    if port_relief:
        put('USBExit_PROBE',box((42,-60,83),(56,-35,99)),None)
        put('ArrayExit_PROBE',box((42,35,83),(56,47,99)),None)
    return parts,meta,path

def pair_audit(parts,changed):
    bounds={n:s.BoundBox for n,s in parts.items()};fails=[];near=[];ncheck=0
    for a,b in itertools.combinations(parts,2):
        if not ({a,b}&changed):continue
        ncheck+=1
        if gap(bounds[a],bounds[b])>2:continue
        d=parts[a].distToShape(parts[b])[0]
        volume=parts[a].common(parts[b]).Volume if gap(bounds[a],bounds[b])<1e-7 else 0
        intended={a,b}=={'ArrayCable_PROBE','ArrayExit_PROBE'}
        row={'a':a,'b':b,'distance_mm':d,'overlap_mm3':volume,'intentional_reservation_union':intended}
        near.append(row)
        if volume>1e-5 and not intended:fails.append(row)
    return {'failed_pairs':fails,'near_pairs':near,'changed_pairs_considered':ncheck,'pass':not fails}

def frame(name,segment):
    if name=='stand':return A.Placement(V(0,0,37+180*math.cos(math.radians(10))),A.Rotation())
    r=next(s['input'] for s in segment['samples'] if 'lift_h'+format(s['h_mm'],'g')==name)
    return A.Placement(V(r['hip_x_mm'],0,r['h_mm']),A.Rotation(V(0,1,0),r['beta_deg']))

def save(name,parts):
    doc=A.newDocument('v25_'+name);mesh=[]
    for n,s in parts.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope')
        o.Scope='Reference mount; actual PCB/wire/fastener interfaces, preload and strength NOT released'
        vis=s
        if 'Motor' in n:
            b=s.BoundBox;vis=box((b.XMin,b.YMin,b.ZMin),(b.XMax,b.YMax,b.ZMax))
        pts,tris=vis.tessellate(1);mesh.append({'name':n,'points':[list(v) for v in pts],'triangles':tris})
    doc.recompute();p=OUT/(name+'.FCStd');doc.saveAs(str(p));Part.export(doc.Objects,str(p.with_suffix('.step')));A.closeDocument(doc.Name)
    doc=A.openDocument(str(p));assert len(doc.Objects)==len(parts) and all(o.Shape.isValid() for o in doc.Objects);A.closeDocument(doc.Name)
    (OUT/(name+'_mesh.json')).write_text(json.dumps(mesh)+'\n')

def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--seat-relief',action='store_true');parser.add_argument('--port-relief',action='store_true');parser.add_argument('--tool-relief',action='store_true');args=parser.parse_args()
    if args.seat_relief:OUT=ROOT/'09_cad/uac_mount_v25_revision'
    if args.port_relief:
        assert args.seat_relief
        OUT=ROOT/'09_cad/uac_mount_v25_ports'
    if args.tool_relief:
        assert args.port_relief
        OUT=ROOT/'09_cad/uac_mount_v25_service'
    OUT.mkdir(exist_ok=True)
    seg=json.loads((ROOT/'04_kinematics/recovery_v22/segment_samples.json').read_text())
    basefile=ROOT/'09_cad/body_v23_fit/integration.json';baseline=json.loads(basefile.read_text())
    sd=A.openDocument(str(ROOT/'09_cad/body_v23_fit/stand.FCStd'))
    native_carrier=placed(sd.getObject('ElectronicsCarrier').Shape,frame('stand',seg).inverse());A.closeDocument(sd.Name)
    parts,meta,path=native(native_carrier,seat_relief=args.seat_relief,port_relief=args.port_relief,tool_relief=args.tool_relief)
    delta_mass=sum(v['mass_kg'] or 0 for v in meta.values())-native_carrier.Volume*cfg['densities_kg_mm3']['PETG_reference']
    assert delta_mass>0
    save('mount_native',parts)
    scenes=[]
    for name in ('stand','lift_h114','lift_h117','lift_h120'):
        T=frame(name,seg);src=ROOT/'09_cad/body_v23_fit'/(name+'.FCStd');doc=A.openDocument(str(src))
        old={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name)
        assembled={**old,**{n:placed(s,T) for n,s in parts.items()}}
        audit=pair_audit(assembled,set(parts));audit.update(scene=name,source_sha256=sha(src))
        audit['ground_penetrations_mm']={n:s.BoundBox.ZMin for n,s in assembled.items() if s.BoundBox.ZMin<-.5}
        audit['pass']=audit['pass'] and not audit['ground_penetrations_mm']
        b=A.BoundBox()
        for s in assembled.values():b.add(s.BoundBox)
        audit['envelope_xyz_mm']=[b.XLength,b.YLength,b.ZLength]
        audit['component_count']=len(assembled)
        save(name,assembled);scenes.append(audit)
        result={'completed':False,'scenes':scenes,'mount_meta':meta,'path':path,'added_CAD_mass_kg':delta_mass,'seat_relief_radius_mm':2.5 if args.seat_relief else None,'port_windows_and_probes':args.port_relief,'tool_relief_length_mm':13 if args.tool_relief else None}
        (OUT/'integration.json').write_text(json.dumps(result,indent=2)+'\n')
        print(name,'failures',audit['failed_pairs'],flush=True)
    masses=[]
    oldm=native_carrier.Volume*cfg['densities_kg_mm3']['PETG_reference']
    for base in baseline['scenes']:
        T=frame(base['scene'],seg)
        support=seg['support_points_mm'] if base['scene']!='stand' else [[x,y,0] for x in (-50,50) for y in (-87.5,-32.5,32.5,87.5)]
        moment=V(*base['COM_mm'])*base['new_mass_kg']-T.multVec(volcenter(native_carrier))*oldm
        for n,s in parts.items():
            if meta[n]['mass_kg'] is not None:moment+=T.multVec(volcenter(s))*meta[n]['mass_kg']
        for extra in cfg['additional_board_wire_mass_probes_kg']:
            mass=base['new_mass_kg']+delta_mass+extra
            com=(moment+T.multVec(V(*cfg['board_space_center_body_mm']))*extra)/mass
            masses.append({'scene':base['scene'],'extra_board_wire_assumption_kg':extra,'mass_with_CAD_mount_kg':mass,'COM_mm':list(com),'margin_mm':margin(support,list(com))})
    result.update(completed=True,input_sha256=sha(PF),baseline_sha256=sha(basefile),mass_sensitivity=masses,
                  actual_board_fit_verified=False,preload_verified=False,CAE_verified=False,manufacturing_release=False)
    (OUT/'integration.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Complete',sum(s['pass'] for s in scenes),'/4; added CAD mass',delta_mass,flush=True)

if __name__=='__main__':main()
