"""Bounded new contact family; preserve original knee90 ground-collision failures."""
import json
import search_recovery_v21 as s
base=json.loads((s.OUT/'forward_extension.json').read_text())['cases'];rows=[]
for r in base:
    if not r['ik_found'] or r['h_mm'] not in (100,120) or r['beta_deg'] not in (30,45,60):continue
    dx=r['left_pose']['foot_translation_mm'][0]-r['hip_x_mm'];dy=r['left_pose']['foot_translation_mm'][1]
    if dx<99 or dy not in (0,15):continue
    p=r['right_pose'];hp=p['q_deg'][2]
    rp=s.old.leg.leg_from_body(r['h_mm'],r['beta_deg'],hp,120,35)['pose'];rp['foot_translation_mm'][0]+=r['hip_x_mm']
    rows.append({**r,'right_pose':rp,'support_knee_replanned_deg':120})
(s.OUT/'knee_replan.json').write_text(json.dumps({'cases':rows,'scope':'Selected forward-step numerical candidates, right support knee120 instead of90; not a transition or global search','preserved_earlier_search':'09_cad/body_v21/leg_screen.json'},indent=2)+'\n',encoding='utf-8')
print('Replanned candidates',len(rows),[r['id'] for r in rows])
