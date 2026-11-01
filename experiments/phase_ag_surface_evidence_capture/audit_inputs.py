"""Read-only local provenance inventory. Output includes private paths; never publish it."""
import argparse,json,csv
from pathlib import Path
from geometry_ieskf.common import dump,sha

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--collection-root',required=True);ap.add_argument('--data-root',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();root=Path(a.collection_root);D=Path(a.data_root);O=Path(a.output);records=[]
 for i,p in enumerate(sorted((root/'map_data').rglob('*.pcd'))):
  header=[]
  with p.open('rb') as f:
   for j in range(50):
    line=f.readline().decode('ascii',errors='replace').strip();header.append(line)
    if line.startswith('DATA '):break
  records.append(dict(id='M'+str(i+1).zfill(2),path=str(p),bytes=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns,sha256=sha(p),pcd_header=header,independent_acquisition='UNVERIFIED',datum_clock_extrinsics='UNVERIFIED'))
 meta=[]
 for p in sorted(root.rglob('metadata.yaml')):
  meta.append(dict(path=str(p),sha256=sha(p),text=p.read_text()))
 reference=[]
 for p in sorted((root/'nmls_full_lap').glob('*truth*.csv')):
  with p.open() as f:
   reader=csv.DictReader(f);rows=list(reader);times=[float(x['timestamp']) for x in rows if x.get('timestamp')]
  reference.append(dict(path=str(p),sha256=sha(p),fields=reader.fieldnames,rows=len(rows),time_range=[min(times),max(times)] if times else [],qualification='DERIVED_REFERENCE_NOT_AUTOMATIC_GROUND_TRUTH'))
 docs=[]
 selected=[root/'map_data/compaer.py',root/'nmls_full_lap/gnss_baselink_truth_delay75_full_validated.csv.validity.txt',root/'nmls_full_lap/analysis/gnss_semantics_audit/gnss_reconstruction_semantics_summary.txt',root/'nmls_full_lap/analysis_validated/experiment_validity.txt',root/'r2_lap_dataset/recording_summary.txt',root/'r2_lap_dataset/analysis_stage1/R2_LAP_STAGE1_REPORT.md',D/'history/phase_v/V4_REFERENCE_AND_CONSISTENCY.md',D/'history/phase_v/V0_TIME_AND_FRAME_AUDIT.md']
 for p in selected:
  if p.is_file():docs.append(dict(path=str(p),sha256=sha(p),text=p.read_text()))
 dump(O/'inventory_private.json',dict(maps=records,bag_metadata=meta,derived_references=reference,provenance_documents=docs,independence='DATA_INSUFFICIENT',no_raw_data_modified=True));print('Inventory complete',len(records),'PCDs',len(meta),'recording metadata',len(reference),'derived reference CSVs')
if __name__=='__main__':main()
