"""Load reviewed v18 definitions without running historic exports or scans."""
import ast
from pathlib import Path
def load_body():
    path=Path(__file__).resolve().with_name('body_integration_v18.py');tree=ast.parse(path.read_text(encoding='utf-8'));statements=[]
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='stand' for t in node.targets):break
        statements.append(node)
    env={'__file__':str(path),'__name__':'body_v18_definitions'};exec(compile(ast.Module(body=statements,type_ignores=[]),str(path),'exec'),env)
    return env
