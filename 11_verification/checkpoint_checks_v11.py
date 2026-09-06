import FreeCAD as A,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
g=json.loads((ROOT/'09_cad/ankle_v11/verification.json').read_text())
assert g['sample_count']==35 and all(not r['penetrations'] for r in g['samples'])
doc=A.openDocument(str(ROOT/'09_cad/ankle_v11/wukong_ankle_structure_v11.FCStd'))
s=doc.RollTransferYoke.Shape;assert s.isValid() and len(s.Solids)==1
r=json.loads((ROOT/'11_verification/bolt_compliance_v11/summary.json').read_text())
assert len(r['results'])==3
for x in r['results']:
    assert x['force_residual_N']<.01 and x['moment_residual_Nmm']<.1
    assert x['energy_relative_error']<1e-4 and x['connector_equilibrium_max_error_N']<.01
for folder in ['v11_transition_baseline','v11_transition_r2']:
    x=json.loads((ROOT/'11_verification'/folder/'mesh_1.8/summary.json').read_text())
    assert x['force_residual_N']<.01 and x['moment_residual_Nmm']<.1 and x['energy_relative_error']<1e-4
out={'stage':'v0.11','checks':[
    {'name':'CAD重新打开且连接架为有效连通实体','status':'passed','evidence':['09_cad/ankle_v11/wukong_ankle_structure_v11.FCStd']},
    {'name':'35姿态所建实体无体积穿透','status':'passed','evidence':['09_cad/ankle_v11/verification.json']},
    {'name':'三组有限刚度模型数值平衡检查','status':'passed','evidence':['11_verification/bolt_compliance_v11/summary.json']},
    {'name':'圆角前后同条件求解数值平衡检查','status':'passed','evidence':['11_verification/v11_transition_baseline/mesh_1.8/summary.json','11_verification/v11_transition_r2/mesh_1.8/summary.json']}
    ],'pending':['真实螺栓预紧/接触与连接刚度实测','局部应力收敛及强度/疲劳','制造放行','完整整机装配/动力学','齐天大圣外观定稿与穿戴验证','真机起身/空翻/递水等能力验证']}
(ROOT/'11_verification/v11_checkpoint_checks.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('PASS: v0.11 geometry/numerical checks; engineering/real-robot gates remain pending.')
