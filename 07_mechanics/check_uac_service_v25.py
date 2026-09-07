"""Stationary service-space probes after unplugging wires/removing cassette screws."""
import json,sys,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from uac_mount_v25 import A,Part,V,cfg,frame,pair_audit,volcenter,sha
OUT=ROOT/'09_cad/uac_mount_v25_ports'

def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--final',action='store_true');args=parser.parse_args()
    if args.final:OUT=ROOT/'09_cad/uac_mount_v25_service'
    source=OUT/'integration.json';result=json.loads(source.read_text())
    assert result['completed'] and all(s['pass'] for s in result['scenes'])
    seg=json.loads((ROOT/'04_kinematics/recovery_v22/segment_samples.json').read_text());T=frame('stand',seg)
    doc=A.openDocument(str(OUT/'stand.FCStd'));parts={o.Name:o.Shape.copy() for o in doc.Objects};A.closeDocument(doc.Name)
    service=[]
    for j in range(4):
        head=parts['UACScrew'+str(j)].BoundBox
        probe=Part.makeCylinder(2,30,V(head.XMax,(head.YMin+head.YMax)/2,(head.ZMin+head.ZMax)/2),V(1,0,0))
        check=pair_audit({**{n:s for n,s in parts.items() if n!='UACScrew'+str(j)},'ToolProbe':probe},{'ToolProbe'})
        service.append({'screw':j,'shaft_diameter_mm':4,'insertion_axis_world':[1,0,0],**check})
    movable={'UACCassette','InsulationSheet_PROVISIONAL','RemainingBoardSpace_PROBE'}
    # Cables are physically unplugged; 4 cassette screws removed before translation.
    retained={n:s for n,s in parts.items() if n not in movable and not n.startswith('UACScrew') and n not in ('ArrayCable_PROBE','USBExit_PROBE','ArrayExit_PROBE')}
    travel=[]
    for dx in range(0,31,2):
        moved={}
        for n in movable:
            moved[n]=parts[n].copy();moved[n].translate(V(dx,0,0))
        check=pair_audit({**retained,**moved},movable)
        travel.append({'withdrawal_x_mm':dx,**check})
    # Geometric cassette screw group only: specified pull-force and inertia probes,
    # not certified impacts or bolt/printed-material capacities.
    loads=[]
    for grams in (20,40,60):
        for accel_g in (1,5,10):
            force=grams/1000*9.81*accel_g
            loads.append({'unmeasured_board_wire_mass_g':grams,'assumed_acceleration_g':accel_g,
                          'inertial_force_N':force,'actual_impact_verified':False})
    result={'completed':True,'integration_sha256':sha(source),'tool_access':service,'withdrawal_samples':travel,
            'sampled_service_geometry_pass':all(s['pass'] for s in service+travel),
            'inertial_force_probes':loads,'USB_pull_test_force_probes_N':[1,5,10],
            'actual_tool_or_hand_envelope_verified':False,'actual_wire_disconnection_verified':False,
            'continuous_sweep_verified':False,'physical_retention_verified':False,
            'scope':'Standing pose only. 4 mm straight tool shaft and 16 translations at 2 mm spacing. Not operating posture, wrench capacity, physical retention or manufacturing approval.'}
    (ROOT/('07_mechanics/uac_service_v25_final.json' if args.final else '07_mechanics/uac_service_v25.json')).write_text(json.dumps(result,indent=2)+'\n')
    print('Service',result['sampled_service_geometry_pass'],'tool',[len(s['failed_pairs']) for s in service],'withdrawal',[len(s['failed_pairs']) for s in travel])

if __name__=='__main__':main()
