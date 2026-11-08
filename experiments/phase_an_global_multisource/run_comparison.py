"""Controlled same-window real-input A/B/C evidence; qualification is not fabricated."""
import argparse,json,time,collections,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.common import dump,sha
from ndt_global.shadow import Contract,PoseHistory
from ndt_global.global_manager import GlobalManager,PositionObservation

def main(a):
    root,work,out,pub=Path(a.data_root),Path(a.work_data),Path(a.private_output),Path(a.public_output)
    out.mkdir(parents=True,exist_ok=True)
    read=lambda p:json.loads(p.read_text())
    data=read(root/'history/phase_v/results/input_continuous_extension.json')
    local=read(work/'phase_al_shadow_map_odom/local_control.json');other=read(work/'phase_al_shadow_map_odom/local_shadow.json')
    al=read(work/'phase_al_shadow_map_odom/decisions.json');am=read(work/'phase_am_ndt_integrity/diagnostics.json');multi=read(work/'phase_am_ndt_integrity/multistart_diagnostics.json');gps=read(out/'gnss_records.json')
    assert len(al)==len(am)==len(data['frames'])==284
    contract=Contract(**al[0]['candidate']['contract']);h=PoseHistory(contract);manager=GlobalManager(h)
    poses={}
    for e in local['outputs']:
        T=np.eye(4);T[:3,:3]=Rotation.from_quat(e['q_map_body_xyzw']).as_matrix();T[:3,3]=e['p_base_map'];poses[e['state_time_ns']]=T
    for e,r in zip(data['frames'],al):poses[e['header_ns']]=np.array(r['local_pose'])
    # Offline scheduler models20ms processing/buffering delay, NOT observed live latency.
    events=[(t,0,'odom',(t,T)) for t,T in poses.items()]
    events.extend((e['header_ns']+20_000_000,1,'ndt',i) for i,e in enumerate(data['frames']))
    events.extend((e['stamp_ns']+20_000_000,2,'gnss',e) for e in gps)
    events.extend((e['state_time_ns']+20_000_000,3,'output',e['state_time_ns']) for e in local['outputs'])
    multi_by_index=dict(zip((0,71,142,213,283),multi));statuses=[];decisions=[];costs=[];publications=0;wire=[]
    for now,_,kind,value in sorted(events,key=lambda e:(e[0],e[1])):
        if kind=='odom':
            h.add(value[0],value[1],contract)
            wire.append(dict(schema_version=1,kind='odom',now_ns=now,stamp_ns=value[0],pose=value[1],contract=contract.__dict__))
            continue
        tick=time.perf_counter_ns()
        if kind=='ndt':
            i=value;e=data['frames'][i];record=am[i];mode=multi_by_index.get(i)
            if mode is not None:
                classification=mode['modes']['classification'];converged=any(c['converged'] and c['candidate_valid'] for c in mode['candidates'])
                candidates=[mode['candidates'][g[0]]['pose'] for g in mode['modes']['clusters']]
            else:classification=record['state'];converged=record['converged'];candidates=[al[i]['candidate']['pose']]
            wire.append(dict(schema_version=1,kind='ndt',now_ns=now,stamp_ns=e['header_ns'],group=e['source_sha256'],candidates=candidates,classification=classification,converged=converged,contract=contract.__dict__))
            decision=manager.ndt(candidates,classification,e['header_ns'],e['source_sha256'],now,contract,converged=converged)
            decisions.append(dict(index=i,kind='ndt',**decision))
        elif kind=='gnss':
            # Raw LLA is NEVER mislabeled as map-base position; conversion/reference
            # qualification failed, so no synthetic map position is constructed.
            g=PositionObservation(value['stamp_ns'],value['source'],value['cdr_sha256'],None,contract,
               fix_valid=False,datum_qualified=False,reference_qualified=False,clock_qualified=False)
            wire.append(dict(schema_version=1,kind='gnss',now_ns=now,observation=g.__dict__|{'contract':contract.__dict__},contract=contract.__dict__))
            decisions.append(dict(kind='gnss',**manager.position(g,now)))
        else:
            wire.append(dict(schema_version=1,kind='output',now_ns=now,stamp_ns=value,contract=contract.__dict__))
            p=manager.publish(value,now);publications+=int(p['published']);statuses.append(p)
        costs.append((time.perf_counter_ns()-tick)/1e6)
    fields=['p_imu_map','v_imu_map','q_map_body_xyzw','bg_body_rad_s','ba_body_m_s2','P']
    for x,y in zip(local['outputs'],other['outputs']):
        for k in fields:np.testing.assert_array_equal(x[k],y[k])
    for t,T in zip(h.stamps,h.poses):np.testing.assert_array_equal(T,poses[t])
    accepted=[r for r in al if r['event']['accepted']];jumps=[];old=None
    for r in accepted:
        C=np.array(r['publication']['correction']);O=np.array(r['local_pose'])
        if old is not None:jumps.append(float(np.linalg.norm((C@O)[:3,3]-(old@O)[:3,3])))
        old=C
    z=[e['p_imu_map'][2] for e in local['outputs']];wheel=[e['delta'] for e in local['wheel_updates'] if e['accepted']]
    base=[e for e in data['events'] if e['kind']=='baseline' and data['begin_ns']<=e['header_ns']<=data['end_ns']]
    states=collections.Counter(s['state'] for s in statuses);reasons=collections.Counter(d['reason'] for d in decisions)
    summary=dict(qualification='PARTIAL_IMPLEMENTATION_NO_GO_QUALIFIED_REAL_FUSION',same_window_frames=284,real_gnss_messages=len(gps),
      comparison={
        'A_recorded_production':{'samples':len(base),'frame_reference_match':all(e['frame_id']=='map' and e['child_frame']=='base_link' for e in base),'map_version_runtime':'UNVERIFIED','NDT_enabled_at_runtime':'UNVERIFIED','acceptance_events':'NOT_RECORDED','accuracy':'NO_INDEPENDENT_REFERENCE'},
        'B_AL_shadow':{'ndt_received':284,'shadow_accepted':155,'trusted_anchors':0,'shadow_published':284,'max_correction_induced_global_jump_m':max(jumps)},
        'C_AN_manager':{'ndt_received':284,'gnss_received':len(gps),'coordinate_anchors':sum(d['accepted'] for d in decisions),'trusted_anchors':0,'global_published':publications,'local_full_state_P_equal':True}},
      observation_reasons=dict(reasons),output_states=dict(states),output_slots=len(statuses),state_available_slots=sum(s['state'] in ['GLOBAL_TRACKING','PARTIAL'] for s in statuses),
      ambiguity_ndt_events=sum(d['reason']=='AMBIGUOUS_NO_ANCHOR' for d in decisions),ambiguous_window_fraction=sum(s['state']=='AMBIGUOUS' for s in statuses)/len(statuses),
      local_vertical={'z_span_m':float(np.ptp(z)),'wheel_max_abs_dZ_m':max(abs(d[2]) for d in wheel),'wheel_max_abs_dvz_m_s':max(abs(d[5]) for d in wheel),'final_sigmaZ_m':float(np.sqrt(local['outputs'][-1]['P'][2][2])),'coupling_removed':False},
      core_ms={'p50':float(np.median(costs)),'p95':float(np.percentile(costs,95)),'max':max(costs)},
      strict_controlled_ABC_qualified=False,qualified_real_recovery=False,GNSS_resolved_real_ndt_modes=False,false_trusted_publications=0,
      actual_gain_or_R_tuning=False,statistical_fusion='UNQUALIFIED_NOT_EXECUTED',end_to_end_20hz=False,
      input_stream_sha256=sha(root/'history/phase_v/results/input_continuous_extension.json'),target_sha256=sha(root/'inputs/map_target.bin'),
      gnss_records_sha256=sha(out/'gnss_records.json'),AL_decisions_sha256=sha(work/'phase_al_shadow_map_odom/decisions.json'),AM_diagnostics_sha256=sha(work/'phase_am_ndt_integrity/diagnostics.json'),AM_multistart_sha256=sha(work/'phase_am_ndt_integrity/multistart_diagnostics.json'))
    dump(out/'ros_events.json',wire);dump(out/'decisions.json',decisions);dump(out/'states.json',statuses);dump(pub/'results.json',summary);print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--work-data',required=True);p.add_argument('--private-output',required=True);p.add_argument('--public-output',required=True);main(p.parse_args())
