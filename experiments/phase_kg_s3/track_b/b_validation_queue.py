"""Bounded B qualification queue; no algorithm changes, GPU ownership via flock.

Only closed COMPLETED runs are gzip archived by root's SHA-verifying helper.
Failed prefixes remain byte-for-byte untouched. The supervising process owns
only its subprocess group; it never interrupts another GPU lock holder.
"""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path


def disk_bytes(root):return sum(p.stat().st_size for p in root.rglob('*') if p.is_file())


def run(plan_path):
    plan=json.loads(Path(plan_path).read_text());root=Path(plan['branch_root']);limit=2*1024**3
    if limit!=plan['branch_limit_bytes']:raise ValueError('FROZEN_B_BRANCH_BUDGET_REQUIRED')
    sys.path.insert(0,plan['shared_module_path']);from archive_closed_log import archive
    while not all(Path(p).exists() for p in plan['wait_for_closed_status']):time.sleep(1.)
    reports=[]
    for row in plan['runs']:
        config=Path(row['config']);cfg=json.loads(config.read_text());out=Path(cfg['output'])
        if not out.resolve().is_relative_to(root.resolve()) or out.exists():raise ValueError('NEW_B_OUTPUT_REQUIRED')
        if disk_bytes(root)>=limit:raise RuntimeError('B_BRANCH_PREFLIGHT_BUDGET_EXCEEDED')
        logfile=Path(row['stdout']);logfile.parent.mkdir(parents=True,exist_ok=True)
        with logfile.open('x') as log:
            process=subprocess.Popen(['flock',plan['gpu_lock'],sys.executable,row['entry'],str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            budget_failure=False
            while process.poll() is None:
                if disk_bytes(root)>limit:
                    budget_failure=True;os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=30)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
                    break
                time.sleep(2.)
        status_path=out/'run_status.json';status=json.loads(status_path.read_text()) if status_path.exists() else dict(reason='NO_CLOSED_RUN_STATUS')
        report=dict(config=str(config),returncode=process.returncode,reason=status['reason'],budget_failure=budget_failure,archives=[])
        if status['reason']=='COMPLETED':
            for name in ['frames.jsonl','kinematic_events.jsonl','input_ledger.jsonl','states.jsonl','kinematic_output100.jsonl','kinematic_output100_receipts.jsonl']:
                p=out/name
                if p.exists() and p.stat().st_size:report['archives'].append(archive(p))
        reports.append(report);Path(plan['status_output']).write_text(json.dumps(dict(completed_queue_entries=reports,remaining=len(plan['runs'])-len(reports)),indent=2));print(json.dumps(report),flush=True)
        if budget_failure:raise RuntimeError('B_BRANCH_RESOURCE_ABORT_PREFIX_RETAINED')
    return reports

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('plan');run(p.parse_args().plan)
