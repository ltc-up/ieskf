"""Offline retained-candidate objective audit; no reference or online change."""
import argparse,json
from collections import Counter
from pathlib import Path
from log_io import open_text

def audit(root):
    counts=Counter();issues=[]
    for line in open_text(Path(root)/'frames.jsonl'):
        frame=json.loads(line)
        if frame.get('no_alignment'):continue
        counts['frames']+=1;update=frame['update']
        candidates=update.get('candidates',update.get('domain',{}).get('candidates',[]))
        labels=[c['label'] for c in candidates]
        if sorted(labels)!=sorted(['prior','align','prior_refined','align_refined']):
            issues.append(dict(ref_ns=frame['ref_ns'],reason='FOUR_CANDIDATE_LABELS',labels=labels))
        valid=[c for c in candidates if 'F' in c and not c.get('failure')]
        for candidate in valid:
            counts['valid_'+candidate['label']]+=1
            if abs(candidate['F']-candidate['prior_contribution']-candidate['geometry_contribution'])>1e-8:
                issues.append(dict(ref_ns=frame['ref_ns'],reason='OBJECTIVE_SUM',label=candidate['label']))
            if abs(candidate['geometry_contribution']+candidate['score'])>1e-8:
                issues.append(dict(ref_ns=frame['ref_ns'],reason='BETA_ONE',label=candidate['label']))
        if update.get('accepted'):
            counts['accepted']+=1
            if not valid or abs(update['selected_metrics']['F']-min(c['F'] for c in valid))>1e-8:
                issues.append(dict(ref_ns=frame['ref_ns'],reason='NOT_MINIMUM_RETAINED_OBJECTIVE'))
    return dict(counts=dict(counts),issues=issues,reference_used=False,qualification='Retained candidate objectives only; not global optimality, statistical calibration or basin correctness')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root');parser.add_argument('--out');args=parser.parse_args()
    result=json.dumps(audit(args.root),indent=2)
    if args.out:Path(args.out).write_text(result+'\n')
    print(result)
