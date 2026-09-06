"""Check consistency of whole-body research outputs; explicitly do not certify tasks."""
import json,math,hashlib,argparse
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v12','v13'],default='v12');args=parser.parse_args();VER=args.variant
ROOT=Path(__file__).resolve().parents[1];HALF=100 if VER=='v13' else 130
data=json.loads((ROOT/'09_cad'/('fullbody_'+VER)/'wholebody_results.json').read_text())
loads=json.loads((ROOT/'05_dynamics'/('fullbody_'+VER)/'task_loads.json').read_text())
records=[]
for p in data['results']:
    assert len(p['joints'])==26 and len({j['name'] for j in p['joints']})==26
    assert len([c for c in p['components'] if c['category']=='motor'])==26
    mass=sum(c['mass_kg'] for c in p['components'])
    assert math.isclose(mass,p['with_costume']['mass_kg'],rel_tol=1e-10)
    cm=sum(c['mass_kg']*np.array(c['com_mm']) for c in p['components'])/mass
    assert np.allclose(cm,p['with_costume']['com_mm'],atol=1e-8)
    for c in p['components']:
        I=np.array(c['inertia_kgm2']);assert np.allclose(I,I.T,atol=1e-10)
        eig=np.linalg.eigvalsh(I);assert eig.min()>=-1e-10 and eig[-1]<=eig[0]+eig[1]+1e-8
    total=p['with_supported_payload'];react=p['contact_reactions'];w=total['mass_kg']*9.81
    forces=[(np.array([react['foot_cop_x_mm'],HALF,0])/1000,np.array([0,0,react['left_foot_N']])),
            (np.array([react['foot_cop_x_mm'],-HALF,0])/1000,np.array([0,0,react['right_foot_N']]))]
    if p['ik']['palms_support']:
        for side in (1,-1):
            point=np.array(p['ik']['arm_target_mm'],dtype=float);point[1]*=side
            forces.append((point/1000,np.array([0,0,react['each_hand_N']])))
    net=sum((f for _,f in forces),np.array([0.,0.,-w]))
    moment=sum((np.cross(x,f) for x,f in forces),np.cross(np.array(total['com_mm'])/1000,[0,0,-w]))
    assert np.linalg.norm(net)<1e-7 and np.linalg.norm(moment)<1e-7
    assert p['ik']['leg_fk_error_mm']<1e-6
    records.append({'pose':p['pose'],'force_residual_N':float(np.linalg.norm(net)),
        'moment_residual_Nm':float(np.linalg.norm(moment)),
        'task_verified':False,'hardware_interference_count':len(p['hardware_interferences'])})
for j in loads['jump']:
    M=loads['wholebody_mass_kg'];v=j['launch_speed_mps'];t=j['push_duration_s'];F=j['total_effective_vertical_force_N']
    assert math.isclose((F-M*9.81)*t,M*v,rel_tol=1e-10)
    assert math.isclose(F*j['reference_com_stroke_m'],j['ideal_push_work_J'],rel_tol=1e-10)
out={'stage':'v0.12','checks':[
    {'name':'26关节/电机数量、分件质量与惯量数学检查','status':'passed','evidence':['09_cad/fullbody_v12/wholebody_results.json']},
    {'name':'五个候选姿态腿长闭合与假定接触静力平衡','status':'passed','evidence':['09_cad/fullbody_v12/wholebody_results.json']},
    {'name':'任务载荷矩阵单位、冲量与能量一致性','status':'passed','evidence':['05_dynamics/fullbody_v12/task_loads.json']}
    ],'records':records,
    'pending':['完整无碰撞装配','腕/夹爪驱动布局','撑地起身IK与机构冲突','主驱动连续/峰值性能','完整动态逆动力学/训练/实测',
               '齐天大圣概念审美确认与可制造外壳','制造放行与整机总价']}
if VER=='v13':
    out['stage']='v0.13'
    for c in out['checks']:
        c['name']=c['name'].replace('五个','七个')
        c['evidence']=[e.replace('fullbody_v12','fullbody_v13') for e in c['evidence']]
    out['pending']=['轴向传动/中心万向架实体设计','夹爪传动和缆索布线','原100mm取瓶区腕膝碰撞','肩部驱动与齐天大圣头壳/耳部干涉',
        '电池/主控/声学方向与供电验证','制造/强度/动态/真机/最终总价']
    selected=['standing','handover','getup_intermediate','pickup_contact_forward_workspace','pickup_lift_forward_workspace']
    for name in selected:
        pose=next(p for p in data['results'] if p['pose']==name)
        assert not pose['hardware_interferences']
    out['checks'].append({'name':'五个离散候选姿态的所建电机/电子包络无穿透','status':'passed','evidence':['09_cad/fullbody_v13/wholebody_results.json']})
(ROOT/'11_verification'/(VER+'_checkpoint_checks.json')).write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(f"PASS: 26-joint count, mass/inertia, {len(data['results'])} conditional force/moment balances, analytical impulse/energy.")
print('NOT PASSED: collision-free assembly, task capability, hardware selection, appearance/manufacturing release.')
