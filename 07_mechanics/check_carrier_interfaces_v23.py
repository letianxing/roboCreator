"""Read saved final CAD and measure selected support/seat/clearance interfaces."""
import json,sys,hashlib
from pathlib import Path
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v23_fit'
pairs=[('ElectronicsCarrier','PelvisPlate','nominal support contact'),('ElectronicsCarrier','CarrierScrew0','nominal countersink seat'),('CarrierScrew0','PelvisPlate','thread omitted; radial envelope gap is not a bolted-joint validation'),
 ('BatteryPadding','battery','nominal padding contact'),('ComputerRetainer0','computer','nominal strap contact'),('PowerRetainer0','power','nominal strap contact'),
 ('HarnessGrommet','PowerHarnessKeepout','unconfirmed cable/grommet clearance'),('ElectronicsCarrier','L_ThighRailm1','moving thigh clearance'),('ComputerRetainer0','microphone','reference microphone clearance')]
result=[]
for name in ('lift_h114','lift_h117','lift_h120','stand'):
    file=OUT/(name+'.FCStd');doc=A.openDocument(str(file));parts={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name)
    checks=[{'a':a,'b':b,'distance_mm':parts[a].distToShape(parts[b])[0],'scope':scope} for a,b,scope in pairs]
    record={'scene':name,'valid_shape_count':sum(s.isValid() for s in parts.values()),'part_count':len(parts),'file_sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'checks':checks}
    if name=='stand':
        import Part
        bb=Part.makeCompound(list(parts.values())).BoundBox;record['bounds_mm']={'size':[bb.XLength,bb.YLength,bb.ZLength],'zmax':bb.ZMax}
    result.append(record);print(name,[(x['a'],x['b'],round(x['distance_mm'],4)) for x in checks],flush=True)
(ROOT/'07_mechanics/carrier_interfaces_v23.json').write_text(json.dumps({'scenes':result,'preload_thread_strength_actual_module_fit_verified':False},indent=2)+'\n',encoding='utf-8')
