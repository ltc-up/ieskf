"""Repack a completed, SHA-indexed gzip log as standard lossless XZ.

Original JSONL bytes are unchanged. Superseded gzip identity stays in manifest;
no original information or failed-run record is discarded.
"""
import argparse,gzip,hashlib,json,lzma,os
from pathlib import Path
from provenance import ROOT
from archive_closed_log import digest_stream


def repack(path):
    path=Path(path)
    if path.is_symlink() or not path.resolve().is_relative_to(ROOT.resolve()) or not path.name.endswith('.jsonl.gz'):raise ValueError('ONLY_AUTHORIZED_CLOSED_GZIP_LOG')
    if json.loads((path.parent/'run_status.json').read_text()).get('reason')!='COMPLETED':raise ValueError('ONLY_COMPLETED_LOGS')
    manifest=path.parent/'lossless_log_archive.json';data=json.loads(manifest.read_text());rows=[r for r in data['files'] if r['archive_file']==path.name]
    if len(rows)!=1:raise ValueError('UNIQUE_SHA_MANIFEST_REQUIRED')
    row=rows[0]
    with path.open('rb') as f:before_digest=digest_stream(f)
    if before_digest!=(row['archive_sha256'],row['archive_bytes']):raise ValueError('GZIP_IDENTITY_MISMATCH')
    target=path.with_suffix('.xz');partial=target.with_name(target.name+'.partial')
    if target.exists() or partial.exists():raise FileExistsError(str(target))
    original_hash=hashlib.sha256();n=0;before=path.stat()
    with gzip.open(path,'rb') as source,partial.open('xb') as raw:
        with lzma.LZMAFile(raw,'w',format=lzma.FORMAT_XZ,preset=6) as dest:
            while block:=source.read(1024**2):original_hash.update(block);n+=len(block);dest.write(block)
        raw.flush();os.fsync(raw.fileno())
    expected=(row['original_sha256'],row['original_bytes'])
    if (original_hash.hexdigest(),n)!=expected:raise ValueError('ORIGINAL_JSONL_IDENTITY_MISMATCH')
    with lzma.open(partial,'rb') as f:
        if digest_stream(f)!=expected:raise ValueError('XZ_ROUNDTRIP_MISMATCH')
    after=path.stat()
    if (before.st_mtime_ns,before.st_size)!=(after.st_mtime_ns,after.st_size):raise ValueError('GZIP_CHANGED')
    with partial.open('rb') as f:sha,size=digest_stream(f)
    partial.rename(target)
    row.setdefault('previous_archives',[]).append({k:row[k] for k in ('archive_file','archive_sha256','archive_bytes')})
    row.update(archive_file=target.name,archive_sha256=sha,archive_bytes=size,compression='XZ preset6',roundtrip_verified=True)
    pending=manifest.with_name(manifest.name+'.partial')
    with pending.open('x') as f:f.write(json.dumps(data,indent=2));f.flush();os.fsync(f.fileno())
    pending.replace(manifest);path.unlink();return row

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path');print(json.dumps(repack(p.parse_args().path)))
