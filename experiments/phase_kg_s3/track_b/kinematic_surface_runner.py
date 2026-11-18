"""New named passive-diagnostic entrypoint; original runner source is untouched."""
import argparse,json
from pathlib import Path
import kinematic_causal_runner as frozen
from passive_surface_diagnostic import probe_class,SurfaceStatus


def run(config):
    cfg=json.loads(Path(config).read_text())
    if cfg.get('surface_diagnostic_variant')!='HISTORICAL_BASE_SOURCE_FACING_PASSIVE':raise ValueError('EXPLICIT_SURFACE_VARIANT_REQUIRED')
    out=Path(cfg['output'])
    if out.exists():raise FileExistsError(str(out))
    stream=None;status=SurfaceStatus();close_error=None;closed=False
    def emit(row):
        nonlocal stream
        if stream is None:stream=frozen.ci.AsyncLog(Path(cfg['output'])/'passive_surface_diagnostics.jsonl')
        stream.put(row);status.record(row)
    original=frozen.FixedHistoryPairProbe;frozen.FixedHistoryPairProbe=probe_class(emit)
    try:frozen.run(config)
    finally:
        frozen.FixedHistoryPairProbe=original
        if stream is not None:
            try:stream.close();closed=True
            except Exception as exc:close_error=repr(exc)
        if out.exists():
            result=status.summary(stream is not None and (out/'passive_surface_diagnostics.jsonl').exists(),closed,0 if stream is None else stream.count,close_error)
            (out/'surface_diagnostic_status.json').write_text(json.dumps(result,indent=2))
        if close_error:raise RuntimeError('SURFACE_SIDECAR_CLOSE_FAILED:'+close_error)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('config');run(p.parse_args().config)
