import json
from rebuild_rollrod_v26 import *
rows=[]
for scene in ['stand','lift_h114','lift_h117','lift_h120']:
    doc=A.openDocument(str(ROOT/'09_cad/uac_mount_v25_service'/(scene+'.FCStd')))
    for name in ['L_RollRod','R_RollRod']:
        old=doc.getObject(name).Shape.copy()
        for ep,sp in [(45,0),(90,0),(45,45),(90,45)]:
            new=rebuild(old,ep,sp);file=OUT/(scene+'_'+name+'_phase'+str(ep)+'_'+str(sp)+'.step');new.exportStep(str(file));again=Part.read(str(file))
            row={'scene':scene,'object':name,'eye_phase_deg':ep,'stem_phase_deg':sp,'native_valid':new.isValid(),
                 'STEP_valid':again.isValid(),'native_delta':delta(old.Solids[0],new.Solids[0]),'STEP_delta':delta(old.Solids[0],again.Solids[0]),'STEP_check':check(again)}
            rows.append(row);print(scene,name,ep,sp,'valid',row['native_valid'],row['STEP_valid'],flush=True)
    A.closeDocument(doc.Name)
(OUT/'seam_trials_all_poses.json').write_text(json.dumps({'complete':True,'rows':rows},indent=2)+'\n')
