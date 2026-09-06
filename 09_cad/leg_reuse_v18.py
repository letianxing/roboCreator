"""Reuse the reviewed v17 geometry definitions without executing its export loop."""
import ast, sys, argparse
from pathlib import Path

def load_v17():
    path=Path(__file__).resolve().with_name('integrate_single_leg_v17.py')
    tree=ast.parse(path.read_text(encoding='utf-8'));body=[]
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='scan' for t in node.targets):break
        # Substitute the argument parser setup; never change process argv or parse host args.
        if isinstance(node,(ast.Assign,ast.Expr)):
            names={n.id for n in ast.walk(node) if isinstance(n,ast.Name)}
            if 'parser' in names or ('args' in names and isinstance(node,ast.Assign)):continue
        body.append(node)
    env={'__file__':str(path),'__name__':'v17_definitions','args':argparse.Namespace(repair_interfaces=True)}
    exec(compile(ast.Module(body=body,type_ignores=[]),str(path),'exec'),env)
    return env

def narrow_v17(env):
    # Form a dogleg in the upper calf crossbar, preserving its connection to side plates.
    b=env['box'];u=env['union'];s=env['src']['FixedFrame']
    s=s.cut(b((0,14,114),(56,4,6)))
    s=u([s,b((0,10,114),(56,4,6)),b((-32,12,114),(8,8,6)),b((32,12,114),(8,8,6))])
    env['src']['FixedFrame']=s;env['oldmass']['FixedFrame']=s.Volume*2.7e-6
    env['cfg']['hip_pitch_outboard_mm']=35

def build_leg(env,pose,variant):
    parts,meta,joints,frames=env['build'](pose,35 if variant=='narrow' else 45)
    if variant=='narrow':
        shape=parts['ThighRail1'].copy()
        # Axes in these sagittal test poses have zero hip/ankle roll/yaw, so +Y is unchanged.
        shape.translate(env['V'](0,-10,0));parts['ThighRail1']=shape
    return parts,meta,joints,frames
