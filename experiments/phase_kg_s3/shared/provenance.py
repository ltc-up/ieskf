"""Private run preflight. No data migration, fitted frame, or reference admission."""
import hashlib, json, os, sqlite3, struct, subprocess
from pathlib import Path

EXPECTED_UUID = os.environ.get('KG_S3_VOLUME_UUID', 'CONFIGURE_AUTHORIZED_VOLUME_UUID')
BASELINE = 'd941720d015dd0826dd6f3c87183b773df7c13db'
VOLUME = Path('/authorized/research_volume')
ROOT = VOLUME / 'autoeagle/phase_kg_s3'
ARCHIVE = VOLUME / 'autoeagle'

def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(4 << 20), b''): h.update(b)
    return h.hexdigest()

def require_run_output(path):
    """Reject symlink escape and storage fallback, independently of name prefix."""
    p = Path(path).resolve()
    if not p.is_relative_to(ROOT.resolve()): raise ValueError('OUTPUT_OUTSIDE_APPROVED_VOLUME')
    raw = subprocess.check_output(['findmnt', '-J', '-T', str(VOLUME), '-o', 'TARGET,OPTIONS,UUID'], text=True)
    row = json.loads(raw)['filesystems'][0]
    if row.get('uuid') != EXPECTED_UUID: raise ValueError('VOLUME_IDENTITY_MISMATCH')
    if 'rw' not in row['options'].split(','): raise ValueError('VOLUME_NOT_READ_WRITE')
    if os.statvfs(VOLUME).f_bavail * os.statvfs(VOLUME).f_frsize < 8 * 1024**3: raise ValueError('STORAGE_RESERVE')
    return row

def cloud_identity(path, expected_sha=None):
    path = Path(path)
    with path.open('rb') as f:
        header = f.read(8)
    if len(header) != 8: raise ValueError('CLOUD_HEADER')
    n, = struct.unpack('<Q', header)
    if path.stat().st_size != 8 + 12*n: raise ValueError('CLOUD_SIZE')
    sha = digest(path)
    if expected_sha and sha != expected_sha: raise ValueError('MAP_IDENTITY_MISMATCH')
    return dict(path=str(path), sha256=sha, points=n, bytes=path.stat().st_size)

def collect():
    repo = ARCHIVE/'phase_be_e0_e4/repository'
    sha = subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()
    if sha != BASELINE: raise ValueError('RESEARCH_BASELINE_CHANGED')
    dirty = subprocess.check_output(['git','-C',str(repo),'status','--porcelain'],text=True)
    if dirty: raise ValueError('RESEARCH_BASE_DIRTY')
    c5 = ARCHIVE/'phase_bg_far_lio/c5'
    cfg = json.loads((c5/'config.frozen.json').read_text())
    p1013 = json.loads((c5/'paths.json').read_text())
    p0105 = json.loads((ARCHIVE/'phase_be_e0_e4/runtime/paths.json').read_text())
    identities = {
      '1013': cloud_identity(p1013['target'],cfg['target_sha']),
      '0105': cloud_identity(p0105['target']),
    }
    bag = Path(cfg['bag'])
    db = sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
    topics = list(db.execute('SELECT id,name,type FROM topics'))
    first = db.execute('SELECT timestamp FROM messages ORDER BY timestamp,id LIMIT 1').fetchone()[0]
    last = db.execute('SELECT timestamp FROM messages ORDER BY timestamp DESC,id DESC LIMIT 1').fetchone()[0]
    db.close()
    return dict(baseline=sha, clean=True, maps=identities,
      bag=dict(path=str(bag),bytes=bag.stat().st_size,mtime_ns=bag.stat().st_mtime_ns,
               recorder_first_ns=first,recorder_last_ns=last,topics=topics,full_bag_rehashed=False),
      interval=dict(start_ns=cfg['start_ns'],init_ns=cfg['init_ns'],end_ns=cfg['end_ns'],
                    origin_ns=cfg['start_ns']-130_000_000_000,
                    pass713_covered_by_recorder=last>=cfg['start_ns']+(713-130)*10**9),
      frozen_c5_config_sha256=digest(c5/'config.frozen.json'),
      shared_input=dict(audit_sha256=digest(cfg['audit']),extrinsics_sha256=digest(cfg['extrinsics']),
                        init_sha256=digest(cfg['init_source'])),
      services={k:dict(path=v,exists=Path(v).is_file(),sha256=digest(v) if Path(v).is_file() else None)
                for k,v in p1013.items() if k in ('ndt','deskew')},
      production_modified=False,geodetic_calibration='UNKNOWN',scope='private environment and identity metadata')

if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args()
    require_run_output(a.output)
    result=collect()
    Path(a.output).parent.mkdir(parents=True,exist_ok=True)
    with Path(a.output).open('x') as f: json.dump(result,f,indent=2)
    print(json.dumps(dict(baseline=result['baseline'],maps={k:v['points'] for k,v in result['maps'].items()},interval=result['interval'])))
