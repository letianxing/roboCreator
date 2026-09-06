"""Rigid power-module packaging trial; sizes/masses retained, mounting pending."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'04_kinematics/recovery_v21'
rows=json.loads((OUT/'knee_midpitch.json').read_text())['cases']
rows=[{**r,'id':r['id']+100,'power_shift_body_mm':[-5,0,0]} for r in rows]
(OUT/'power_refit.json').write_text(json.dumps({'cases':rows,'power_change':{'translation_body_mm':[-5,0,0],'size_changed':False,'mass_changed':False,'mounting_and_cabling_validated':False},'scope':'Packaging research, no physical or production change'},indent=2)+'\n',encoding='utf-8')
