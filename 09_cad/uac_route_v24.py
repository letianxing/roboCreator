"""Route-space exploration; rectilinear clearance corridors are NOT flexible cables."""
import json
import math
import argparse
from uac_space_v24 import A, Part, V, ROOT, OUT, box, placed, audit, sha


def corridor(points, halfwidth=2):
    result={}
    for i,(a,b) in enumerate(zip(points,points[1:])):
        assert sum(abs(x-y)>1e-8 for x,y in zip(a,b))==1
        center=[(x+y)/2 for x,y in zip(a,b)]
        dims=[abs(x-y)+2*halfwidth for x,y in zip(a,b)]
        result['ArrayRouteZone'+str(i)]=box(center,dims)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--upper',action='store_true');args=parser.parse_args()
    search=json.loads((OUT/'search.json').read_text())
    assert search['completed'] and search['selected_candidate']['id']==1
    seg=json.loads((ROOT/'04_kinematics/recovery_v22/segment_samples.json').read_text())
    cases={}
    for name in ('stand','lift_h114','lift_h117','lift_h120'):
        if name=='stand':
            T=A.Placement(V(0,0,37+180*math.cos(math.radians(10))),A.Rotation())
        else:
            r=next(s['input'] for s in seg['samples'] if 'lift_h'+format(s['h_mm'],'g')==name)
            T=A.Placement(V(r['hip_x_mm'],0,r['h_mm']),A.Rotation(V(0,1,0),r['beta_deg']))
        doc=A.openDocument(str(OUT/(name+'.FCStd')))
        parts={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')}
        A.closeDocument(doc.Name)
        cases[name]=(parts,{n:s.BoundBox for n,s in parts.items()},T)
    routes=[]
    for side in (49,55,61):
        for z in ((101,105) if args.upper else (55,59)):
            # Initial endpoint is the generic array-exit bay, not an identified pin/socket.
            pts=[[49,47,91],[49,side,91],[49,side,z],[-76,side,z],[-76,side,83],[-76,0,83]]
            native=corridor(pts)
            scenes=[]
            for name,(parts,bounds,T) in cases.items():
                probe={n:placed(s,T) for n,s in native.items()}
                # Intentional connection to Array_Exit reservation only. All 101 old parts
                # and the UAC / USB reservations remain obstacles.
                obstacles={n:s for n,s in parts.items() if n!='Array_Exit_RESERVATION'}
                check=audit(probe,obstacles,bounds,2)
                check['scene']=name
                check['ground_clearance_mm']=min(s.BoundBox.ZMin for s in probe.values())
                check['pass']=check['pass'] and check['ground_clearance_mm']>=2
                scenes.append(check)
            row={'id':len(routes),'waypoints_body_mm':pts,'square_cross_section_mm':4,
                 'polyline_length_mm':sum((V(*a)-V(*b)).Length for a,b in zip(pts,pts[1:])),
                 'scenes':scenes,'pass':all(s['pass'] for s in scenes)}
            routes.append(row)
            print('Route',row['id'],[len(s['failed_pairs']) for s in scenes],flush=True)
    passing=[r for r in routes if r['pass']]
    selected=min(passing,key=lambda r:r['polyline_length_mm']) if passing else None
    if selected:
        native=corridor(selected['waypoints_body_mm'])
        for name,(parts,bounds,T) in cases.items():
            allparts={**parts,**{n:placed(s,T) for n,s in native.items()}}
            doc=A.newDocument('route_'+name)
            for n,s in allparts.items():
                o=doc.addObject('PartDesign::Feature',n);o.Shape=s
                o.addProperty('App::PropertyString','Scope');o.Scope='SPACE RESERVATION ONLY; no measured board, actual connectors or bend-radius compliance'
            doc.recompute();path=OUT/('routed_'+name+'.FCStd');doc.saveAs(str(path))
            Part.export(doc.Objects,str(OUT/('routed_'+name+'.step')));A.closeDocument(doc.Name)
            check=A.openDocument(str(path));assert len(check.Objects)==109 and all(o.Shape.isValid() for o in check.Objects)
            A.closeDocument(check.Name)
            mesh=[]
            for n,s in allparts.items():
                # Exact tessellation of reference shapes; motors simplified for preview only.
                if 'Motor' in n:
                    b=s.BoundBox;s=Part.makeBox(b.XLength,b.YLength,b.ZLength,V(b.XMin,b.YMin,b.ZMin))
                p,t=s.tessellate(1)
                mesh.append({'name':n,'points':[list(v) for v in p],'triangles':t})
            (OUT/('routed_'+name+'_mesh.json')).write_text(json.dumps(mesh)+'\n')
    result={'completed':True,'search_sha256':sha(OUT/'search.json'),'routes':routes,'selected_route':selected,
            'assumptions':{'corridor_section_mm':[4,4],'external_clearance_mm':2,'end_socket_unknown':True},
            'actual_cable_length_mm':None,'actual_bend_radius_mm':None,'electrical_length_limit_verified':False,
            'strain_relief_verified':False,'actual_cable_routing_verified':False,'manufacturing_release':False}
    (OUT/('routing_upper.json' if args.upper else 'routing.json')).write_text(json.dumps(result,indent=2)+'\n')
    print('ROUTE SELECTED',selected['id'] if selected else None,flush=True)


if __name__=='__main__':main()
