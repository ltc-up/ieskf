"""Streaming full-denominator numerical-domain summary for continuous frames."""
from log_io import open_text
import argparse,json
from collections import Counter

def summarize(path):
    n=0;whole=0;any_count=0;axes=Counter();reasons=Counter();pair_counts=[];scores=[];scores_per_pair=[]
    tracks={key:dict(start=None,last=None,max_span_s=0,intervals=0) for key in ['whole_scan_prediction_only','any_candidate_or_trial_domain_rejection']}
    for line in open_text(path):
        f=json.loads(line)
        if f.get('no_alignment'):continue
        n+=1;u=f.get('update',{});stamp=f['ref_ns']
        nested=u.get('domain') or {}
        events=u.get('domain_rejections',nested.get('domain_rejections',[]))
        is_whole=u.get('reason')=='NUMERICAL_DOMAIN_INVALID' and not u.get('accepted',False)
        has=bool(events) or is_whole
        whole+=is_whole;any_count+=has
        for event in events:
            axes.update(event.get('axes',[]));reasons.update([event.get('reason','UNSPECIFIED')])
        metrics=u.get('selected_metrics',{})
        if 'valid_pairs' in metrics:pair_counts.append(metrics['valid_pairs'])
        if 'score' in metrics:scores.append(metrics['score'])
        if metrics.get('valid_pairs',0)>0:scores_per_pair.append(metrics['score']/metrics['valid_pairs'])
        for key,active in [('whole_scan_prediction_only',is_whole),('any_candidate_or_trial_domain_rejection',has)]:
            t=tracks[key]
            if active:
                if t['start'] is None:t['start']=stamp;t['intervals']+=1
                t['last']=stamp;t['max_span_s']=max(t['max_span_s'],(stamp-t['start'])*1e-9)
            else:t['start']=None;t['last']=None
    for track in tracks.values():track.pop('start');track.pop('last')
    def stats(a):
        if not a:return None
        import numpy as np
        return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(max(a)),min=float(min(a)))
    return dict(frames=n,all_domain_frames_retained_in_denominator=True,whole_scan_prediction_only=whole,whole_scan_rejection_fraction=whole/n if n else None,frames_with_any_domain_rejection=any_count,any_domain_fraction=any_count/n if n else None,axis_trigger_counts=dict(axes),reason_counts=dict(reasons),contiguous_reference_time_spans=tracks,selected_valid_pairs=stats(pair_counts),selected_raw_score=stats(scores),selected_score_per_valid_pair_diagnostic_only=stats(scores_per_pair),statistically_calibrated=False)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('frames');p.add_argument('--out');a=p.parse_args();text=json.dumps(summarize(a.frames),indent=2)
 if a.out:open(a.out,'w').write(text+'\n')
 print(text)
