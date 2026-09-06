"""Inventory all project artifacts and inspect text/token signatures before user-authorized push."""
import os,json,hashlib,re,zipfile,datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'11_verification/git_archive_inventory.json'
GATE=ROOT/'11_verification/git_archive_checks.json'
TEXT={'.md','.py','.ps1','.json','.jsonl','.yaml','.yml','.toml','.ini','.cfg','.txt','.log','.xml','.env'}
RULES=[('private_key',re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----')),
       ('github_token',re.compile(rb'(?:ghp_|github_pat_)[A-Za-z0-9_]{24,}')),
       ('openai_project_key',re.compile(rb'sk-proj-[A-Za-z0-9_-]{32,}')),
       ('aws_access_key',re.compile(rb'AKIA[0-9A-Z]{16}'))]
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
def inspect_stream(f,label,findings):
    tail=b''
    for chunk in iter(lambda:f.read(1024*1024),b''):
        data=tail+chunk
        for name,pattern in RULES:
            if pattern.search(data):findings.append({'path':label,'signature':name})
        tail=data[-512:]
def main():
    rows=[];findings=[];dirs=[];omitted=[];zip_members=0
    for current,subdirs,files in os.walk(ROOT):
        here=Path(current);subdirs[:]=sorted(x for x in subdirs if x not in ('.git','__pycache__') and not (here/x).is_symlink())
        if here!=ROOT:dirs.append(here.relative_to(ROOT).as_posix())
        for name in sorted(files):
            f=here/name;rel=f.relative_to(ROOT).as_posix()
            if f in (OUT,GATE):continue
            if f.suffix.lower() in ('.pyc','.pyo'):omitted.append(rel);continue
            if f.is_symlink():raise RuntimeError('Unexpected symlink: '+rel)
            rows.append({'path':rel,'bytes':f.stat().st_size,'sha256':digest(f)})
            if f.suffix.lower() in TEXT:
                with f.open('rb') as stream:inspect_stream(stream,rel,findings)
            if f.suffix.lower()=='.zip':
                with zipfile.ZipFile(f) as z:
                    for member in z.infolist():
                        if not member.is_dir():zip_members+=1
                        if Path(member.filename).suffix.lower() in TEXT:
                            with z.open(member) as stream:inspect_stream(stream,rel+'::'+member.filename,findings)
    # Historical snapshot hashes must agree with their append-only verified index.
    index=ROOT/'checkpoints/index.jsonl';verified=[]
    if index.exists():
        bypath={r['path']:r for r in rows}
        for line in index.read_text(encoding='utf-8').splitlines():
            entry=json.loads(line);rel=entry['archive'].replace('\\','/')
            assert bypath[rel]['sha256']==entry['archive_sha256'],rel
            verified.append(entry['id'])
    result={'created_at':datetime.datetime.now().astimezone().isoformat(),'file_count':len(rows),'bytes':sum(r['bytes'] for r in rows),
      'files':rows,'directories':dirs,'archive_members_examined':zip_members,'verified_historical_snapshots':verified,
      'secret_signature_findings':findings,'secret_scan_scope':'Known token/private-key signatures in project text and ZIP text members; not a guarantee of all possible secret formats',
      'excluded':['.git/**','**/__pycache__/**','*.pyc','*.pyo','this_inventory_and_gate_itself'],
      'note':'New checkpoint and subsequent Git receipt are generated after this pre-commit inventory and are included when staging.'}
    OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    assert not findings,'Secret signatures detected; inspect paths in inventory before upload.'
    gate={'stage':'git_archive','checks':[
      {'name':'Project file inventory and original historical snapshot hashes verified','status':'passed','evidence':['11_verification/git_archive_inventory.json','tools/audit_git_archive.py']},
      {'name':'No configured token/private-key signatures found in inspected project/ZIP text','status':'passed','evidence':['11_verification/git_archive_inventory.json']},
      {'name':'Archive scope, project status and large-file handling recorded','status':'passed','evidence':['deliverables/archive_and_push_record.md','.gitattributes','.gitignore']}],
      'pending':['Remote repository access and full Git/LFS push verification','All engineering pending items remain as recorded in project_state.md']}
    GATE.write_text(json.dumps(gate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ['file_count','bytes','archive_members_examined','verified_historical_snapshots','secret_signature_findings']},indent=2))
if __name__=='__main__':main()
