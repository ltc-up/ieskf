"""Lossless closed-log archival with byte-identity verification before release.

Only an explicitly named closed JSONL under the authorized experiment root is
accepted. Keep the exact uncompressed SHA/size, gzip SHA/size and atomic manifest.
The original pathname is restored by ordinary gzip decompression; transparent
offline readers also accept the original pathname with a .gz fallback.
"""
import argparse, gzip, hashlib, json, os
from pathlib import Path
from provenance import ROOT


def digest_stream(stream):
    h=hashlib.sha256();n=0
    while block:=stream.read(1024**2):h.update(block);n+=len(block)
    return h.hexdigest(),n


def archive(path):
    path=Path(path)
    if path.is_symlink() or not path.resolve().is_relative_to(ROOT.resolve()) or path.suffix!='.jsonl':
        raise ValueError('ONLY_REGULAR_AUTHORIZED_JSONL')
    status=path.parent/'run_status.json'
    if not status.exists():raise ValueError('CLOSED_RUN_STATUS_REQUIRED')
    if json.loads(status.read_text()).get('reason')!='COMPLETED':raise ValueError('ONLY_COMPLETED_RUN_LOGS')
    target=path.with_name(path.name+'.gz');temporary=path.with_name(path.name+'.gz.partial')
    if target.exists() or temporary.exists():raise FileExistsError(str(target))
    before=path.stat();h=hashlib.sha256();n=0
    with path.open('rb') as source,temporary.open('xb') as raw:
        with gzip.GzipFile(filename='',mode='wb',fileobj=raw,compresslevel=3,mtime=0) as dest:
            while block:=source.read(1024**2):h.update(block);n+=len(block);dest.write(block)
        raw.flush();os.fsync(raw.fileno())
    after=path.stat()
    if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):raise ValueError('SOURCE_CHANGED_DURING_ARCHIVAL')
    with gzip.open(temporary,'rb') as f:roundtrip=digest_stream(f)
    original=(h.hexdigest(),n)
    if roundtrip!=original:raise ValueError('UNCOMPRESSED_BYTE_IDENTITY_MISMATCH')
    with temporary.open('rb') as f:compressed=digest_stream(f)
    temporary.rename(target)
    manifest=path.parent/'lossless_log_archive.json'
    data=json.loads(manifest.read_text()) if manifest.exists() else dict(kind='SHA-verified lossless closed-log archive',files=[])
    row=dict(original_relative_path=path.name,original_sha256=original[0],original_bytes=original[1],archive_file=target.name,archive_sha256=compressed[0],archive_bytes=compressed[1],roundtrip_verified=True)
    data['files'].append(row);pending=manifest.with_name(manifest.name+'.partial')
    with pending.open('x') as f:f.write(json.dumps(data,indent=2));f.flush();os.fsync(f.fileno())
    pending.replace(manifest)
    # Every byte is now retained in verified gzip and indexed by the manifest.
    path.unlink()
    return row

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path',type=Path);a=p.parse_args();print(json.dumps(archive(a.path)))
