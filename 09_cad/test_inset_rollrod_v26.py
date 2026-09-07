"""Test geometry-equivalent reduction of buried Boolean overlap, all target poses."""
import json
from rebuild_rollrod_v26 import *

def main():
    rows=[]
    for scene in ['stand','lift_h114','lift_h117','lift_h120']:
        doc=A.openDocument(str(ROOT/'09_cad/uac_mount_v25_service'/(scene+'.FCStd')))
        for name in ['L_RollRod','R_RollRod']:
            old=doc.getObject(name).Shape.copy()
            for inset in [3,4]:
                new=rebuild(old,0,0,inset);file=OUT/(scene+'_'+name+'_inset'+str(inset)+'.step');new.exportStep(str(file));again=Part.read(str(file))
                row={'scene':scene,'object':name,'construction_inset_mm':inset,'native_valid':new.isValid(),
                     'STEP_valid':again.isValid(),'native_delta':delta(old.Solids[0],new.Solids[0]),'STEP_delta':delta(old.Solids[0],again.Solids[0]),'STEP_check':check(again)}
                rows.append(row);print(scene,name,inset,'valid',row['native_valid'],row['STEP_valid'],'volume delta',row['native_delta']['volume_delta_mm3'],flush=True)
        A.closeDocument(doc.Name)
    (OUT/'inset_trials.json').write_text(json.dumps({'complete':True,'rows':rows},indent=2)+'\n')
if __name__=='__main__':main()
