"""Small synthetic contract tests; not localization or statistical qualification."""
import json,time
import numpy as np
import pytest
from continuous_inputs import Prefetch,AsyncLog,START,INIT,END

def test_prefetch_order_and_error():
 def seq():
  for i in range(15):yield dict(kind='imu',id=i)
  raise ValueError('explicit_source_failure')
 p=Prefetch(seq())
 try:
  assert [p.get()['id'] for _ in range(15)]==list(range(15))
  with pytest.raises(RuntimeError,match='explicit_source_failure'):p.get()
 finally:p.close()

def test_prefetch_memory_accounting():
 def seq():
  for i in range(30):yield dict(kind='cloud',id=i,data=np.ones(1024,dtype='f4'))
 p=Prefetch(seq())
 try:
  for i in range(30):assert p.get()['id']==i
  assert p.peak<=p.maxbytes
 finally:p.close()

def test_async_logger_preserves_identity(tmp_path):
 p=tmp_path/'log.jsonl';w=AsyncLog(p)
 for i in range(20):w.put(dict(id=i,P=np.eye(3)))
 w.close();x=[json.loads(l) for l in p.read_text().splitlines()];assert [r['id'] for r in x]==list(range(20));assert x[0]['P']==np.eye(3).tolist()

def test_fixed_time_order():
 assert START<INIT<END
 assert INIT-START==5_000_000_000
 assert (END-START)/1e9==pytest.approx(215.80606308)
