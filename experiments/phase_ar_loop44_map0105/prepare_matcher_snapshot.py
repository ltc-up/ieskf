"""Apply one verified math correction in a NEW authorized private source snapshot.
No restricted implementation is vendored. Existing production/snapshots untouched.
"""
import argparse,hashlib,json,shutil
from pathlib import Path

def prepare(source,output,expected_sha):
 source=source.resolve();output=output.resolve();p=source/'cuda/ndt_linearize.cu'
 if output.exists() or source==output or source in output.parents:raise ValueError('NEW_SEPARATE_OUTPUT_REQUIRED')
 actual=hashlib.sha256(p.read_bytes()).hexdigest()
 if actual!=expected_sha:raise ValueError('SOURCE_IDENTITY_MISMATCH')
 old='-cy * cz * x + cy * sz * y + sy * z';new='-cy * cz * x + cy * sz * y - sy * z'
 text=p.read_text()
 if text.count(old)!=1:raise ValueError('UNEXPECTED_EXPRESSION_COUNT')
 shutil.copytree(source,output);q=output/'cuda/ndt_linearize.cu';q.write_text(text.replace(old,new))
 return dict(original_sha256=actual,corrected_sha256=hashlib.sha256(q.read_bytes()).hexdigest(),changed_expression_count=1,production_modified=False)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--new-private-output',type=Path,required=True);p.add_argument('--expected-source-sha256',required=True);a=p.parse_args();print(json.dumps(prepare(a.source,a.new_private_output,a.expected_source_sha256),indent=2))
