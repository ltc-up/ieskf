"""Transparent JSON/JSONL gzip reading and SHA-checked restoration, no deletion."""
import argparse,gzip,hashlib,json
from pathlib import Path


def open_text(path):
    path=Path(path)
    if not path.exists() and path.with_name(path.name+'.gz').exists():path=path.with_name(path.name+'.gz')
    return gzip.open(path,'rt',encoding='utf-8') if path.suffix=='.gz' else path.open(encoding='utf-8')


def read_json(path):
    with open_text(path) as f:return json.load(f)


def iter_jsonl(path):
    with open_text(path) as f:
        for line in f:yield json.loads(line)


def restore(manifest_path,original_relative_path,destination):
    manifest_path=Path(manifest_path);m=read_json(manifest_path)
    row=next(r for r in m['files'] if r['original_relative_path']==original_relative_path)
    source=manifest_path.parent/row['archive_file'];destination=Path(destination)
    h=hashlib.sha256();n=0
    opener=gzip.open if source.suffix=='.gz' else open
    with opener(source,'rb') as src,destination.open('xb') as dst:
        while True:
            block=src.read(1024**2)
            if not block:break
            dst.write(block);h.update(block);n+=len(block)
    if h.hexdigest()!=row['original_sha256'] or n!=row['original_bytes']:raise ValueError('RESTORE_BYTE_IDENTITY_MISMATCH')
    return dict(restored_bytes=n,sha256=h.hexdigest())

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('manifest');a.add_argument('original_relative_path');a.add_argument('destination');x=a.parse_args();print(json.dumps(restore(x.manifest,x.original_relative_path,x.destination)))
