"""Small local revisions, preserving the original v23 draft and its failed integration."""
import json
import carrier_v23 as c
PF=c.ROOT/'03_system/carrier_revision_v23.json'

FIT=c.ROOT/'03_system/carrier_fit_v23.json'
def native(fit=False):
    config=json.loads(PF.read_text());f=json.loads(FIT.read_text()) if fit else None
    parts,meta,harness=c.native(countersink_clearance=f['countersink_radial_clearance_mm'] if fit else .15);V=c.V;Part=c.Part;size=config['front_lower_edge_chamfer_mm']
    x,z=39.4,54
    loop=Part.makePolygon([V(x-size,-23,z),V(x+.01,-23,z),V(x+.01,-23,z+size+.01),V(x-size,-23,z)])
    wedge=Part.Face(loop).extrude(V(0,46,0));before=parts['ElectronicsCarrier'];parts['ElectronicsCarrier']=before.cut(wedge).removeSplitter()
    hole=Part.makeCylinder(config['grommet_inner_radius_mm'],7,V(-36,-40,45));parts['HarnessGrommet']=parts['HarnessGrommet'].cut(hole).removeSplitter()
    if fit:
        parts['PelvisPlate']=parts['PelvisPlate'].cut(Part.makeCylinder(f['grommet_outer_radius_mm'],6,V(-36,-40,45))).removeSplitter()
        parts['HarnessGrommet']=Part.makeCylinder(f['grommet_outer_radius_mm'],4,V(-36,-40,46)).fuse(Part.makeCylinder(f['grommet_flange_radius_mm'],1,V(-36,-40,50))).cut(Part.makeCylinder(f['grommet_inner_radius_mm'],7,V(-36,-40,45))).removeSplitter()
    for name in ('ElectronicsCarrier','HarnessGrommet','PelvisPlate'):
        shape=parts[name];assert shape.isValid() and len(shape.Solids)==1,name
        meta[name]['mass_kg']=shape.Volume*c.rho[meta[name]['material_assumption']]
    harness['revision']={'chamfer_mm':size,'grommet_inner_diameter_mm':2*config['grommet_inner_radius_mm']}
    if fit:harness['revision'].update(grommet_inner_diameter_mm=2*f['grommet_inner_radius_mm'],grommet_plate_bore_mm=2*f['grommet_outer_radius_mm'],nominal_screw_seating=True)
    return parts,meta,harness
