"""Audit candidate references; a filename containing truth/validated is not authority."""
import argparse,csv,json,hashlib
from pathlib import Path

def run(root):
 p=root/'gnss_baselink_truth_delay75_full_validated.csv';rows=list(csv.DictReader(p.open()));times=[float(r['timestamp']) for r in rows];valid=[r for r in rows if r['truth_valid']=='1'];delay=sorted(set(float(r['gps_delay_correction_ms']) for r in rows));return dict(rows=len(rows),flag_valid_rows=len(valid),duration_s=max(times)-min(times),gnss_delay_corrections_ms=delay,csv_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),reference_independent=False,qualified_accuracy=False,qualification_reasons=['derived from same raw GNSS, not an independent receiver','75ms correction provenance/calibration not established','fix8 interpretation not verified for actual model/firmware','lever, attitude, map datum and height convention not verified','interpolation validity flags do not establish metrological accuracy'],historical_lap_window_duration_s=73.434533834,historical_lap_window_qualification='prior derived window; not a surveyed lap certificate or new accuracy-based selection')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--reference-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(run(a.reference_dir),indent=2)+'\n')
