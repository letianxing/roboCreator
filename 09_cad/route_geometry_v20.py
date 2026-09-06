"""Reuse v19 geometry, without executing old validation/export loops."""
import ast, importlib.util, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
def load():
    path=ROOT/'09_cad/assemble_recovery_v19.py';spec=importlib.util.spec_from_file_location('assembly19',path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    tree=ast.parse(path.read_text(encoding='utf-8'));fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='build');keep=[]
    for node in fn.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='collisions' for t in node.targets):break
        keep.append(node)
    fn.name='build_geometry';fn.body=keep+[ast.Return(value=ast.Tuple(elts=[ast.Name(id=n,ctx=ast.Load()) for n in ['whole','info','joints','centers','cuff_points','palm_points']],ctx=ast.Load()))]
    module=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]));exec(compile(module,str(path),'exec'),mod.__dict__)
    return mod
