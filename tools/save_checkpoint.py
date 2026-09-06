"""Append-only full project snapshots with per-entry SHA-256 verification."""
import argparse,datetime,hashlib,json,re,zipfile,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',required=True);p.add_argument('--gate-file',required=True);args=p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+',args.stage):raise ValueError('Invalid stage label')
    gatepath=(ROOT/args.gate_file).resolve();assert gatepath.is_relative_to(ROOT)
    gates=json.loads(gatepath.read_text(encoding='utf-8'))
    passed=[g for g in gates['checks'] if g['status']=='passed'];assert passed,'No passed check evidence'
    for g in passed:
        assert g['evidence']
        for rel in g['evidence']:
            path=(ROOT/rel).resolve();assert path.is_relative_to(ROOT) and path.is_file(),rel
    stamp=datetime.datetime.now().astimezone();ident=stamp.strftime('%Y%m%d_%H%M%S_%f')+'_'+args.stage
    dest=ROOT/'checkpoints'/ident;dest.mkdir(parents=True,exist_ok=False)
    paths=[];dirs=[]
    for current,subdirs,files in os.walk(ROOT,followlinks=False):
        here=Path(current)
        subdirs[:]=sorted(d for d in subdirs if d not in ('__pycache__','.git') and not (here==ROOT and d=='checkpoints') and not (here/d).is_symlink())
        if here!=ROOT:dirs.append(here.relative_to(ROOT).as_posix()+'/')
        for name in sorted(files):
            f=here/name
            if f.is_symlink():raise RuntimeError('Snapshot will not follow symlink: '+str(f))
            paths.append(f)
    manifest={'id':ident,'created_at':stamp.isoformat(),'stage':args.stage,'gates':gates,
        'excluded':['checkpoints/','**/__pycache__/','**/.git/'],'files':[],'empty_directories_preserved':True}
    partial=dest/'project.partial.zip';final=dest/'project.zip'
    with zipfile.ZipFile(partial,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
        for d in dirs:z.writestr(d,b'')
        for i,f in enumerate(paths):
            rel=f.relative_to(ROOT).as_posix();before=f.stat();sha=hashlib.sha256()
            with f.open('rb') as src,z.open(rel,'w',force_zip64=True) as out:
                for block in iter(lambda:src.read(4*1024*1024),b''):sha.update(block);out.write(block)
            after=f.stat()
            if before.st_size!=after.st_size or before.st_mtime_ns!=after.st_mtime_ns:
                raise RuntimeError('File changed during snapshot: '+rel)
            manifest['files'].append({'path':rel,'bytes':before.st_size,'sha256':sha.hexdigest()})
            if i%75==0:print('Archived',i+1,'/',len(paths),flush=True)
        z.writestr('_snapshot_manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2).encode('utf-8'))
    print('Verifying each archived file...',flush=True)
    with zipfile.ZipFile(partial) as z:
        for row in manifest['files']:
            sha=hashlib.sha256()
            with z.open(row['path']) as f:
                for block in iter(lambda:f.read(4*1024*1024),b''):sha.update(block)
            assert sha.hexdigest()==row['sha256'],row['path']
    partial.rename(final)
    (dest/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    summary={'id':ident,'stage':args.stage,'verified':True,'file_count':len(paths),
        'source_bytes':sum(x['bytes'] for x in manifest['files']),'archive_bytes':final.stat().st_size,
        'archive_sha256':digest(final),'archive':str(final.relative_to(ROOT)),
        'passed_checks':[g['name'] for g in passed],'pending':gates.get('pending',[])}
    (dest/'verification.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (ROOT/'checkpoints/index.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(summary,ensure_ascii=False)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
if __name__=='__main__':main()
