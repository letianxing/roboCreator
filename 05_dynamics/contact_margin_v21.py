"""Support-polygon margin and feasible contact-force ranges; no dynamic stability claim."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'05_dynamics/support_v21';OUT.mkdir(exist_ok=True)

def assess(points,com,mass):
    p=np.array(points)[:,:2];c=np.array(com)[:2];hull=ConvexHull(p);margins=-(hull.equations[:,:2]@c+hull.equations[:,2])
    A=np.vstack([np.ones(len(p)),p.T/1000]);b=mass*9.81*np.r_[1,c/1000]
    ranges=[]
    for i in range(len(p)):
        obj=np.eye(len(p))[i];lo=linprog(obj,A_eq=A,b_eq=b,bounds=(0,None),method='highs');hi=linprog(-obj,A_eq=A,b_eq=b,bounds=(0,None),method='highs')
        ranges.append({'point_index':i,'minimum_force_N':float(lo.fun) if lo.success else None,'maximum_force_N':float(-hi.fun) if hi.success else None})
    bounds=[(0,None)]*len(p);bounds[-1]=(1,None)
    palm=linprog(np.zeros(len(p)),A_eq=A,b_eq=b,bounds=bounds,method='highs')
    return {'support_polygon_margin_mm':float(min(margins)),'hull_vertex_indices':hull.vertices.tolist(),'force_ranges':ranges,'right_palm_at_least_1N_feasible':bool(palm.success),
            'scope':'Flat rigid ground, vertical gravity and unilateral point reactions only. 1N is a diagnostic probe, not a required or commanded hardware load.'}

def main():
    source=json.loads((ROOT/'09_cad/body_v21/single_palm_extended.json').read_text());rows=[]
    for r in source['selected']:
        rows.append({'scene':r['name'],'h_mm':r['input']['h_mm'],**assess(r['support_points_mm'],r['COM_mm'],r['mass_kg'])})
    lift=json.loads((ROOT/'09_cad/body_v21/local_lift.json').read_text())
    for r in lift.get('samples',[]):
        if r.get('geometry_checked'):rows.append({'scene':'local_lift','h_mm':r['h_mm'],'geometry_and_static_pass':r['sample_geometry_and_static_pass'],**assess(r['support_points_mm'],r['COM_mm'],r['mass_kg'])})
    (OUT/'contact_margins.json').write_text(json.dumps({'cases':rows,'dynamic_stability_verified':False},indent=2)+'\n',encoding='utf-8')
    print([(r['scene'],r['h_mm'],round(r['support_polygon_margin_mm'],3),r['right_palm_at_least_1N_feasible']) for r in rows])
if __name__=='__main__':main()
