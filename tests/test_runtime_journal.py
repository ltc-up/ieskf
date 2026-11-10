import unittest,tempfile,sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'experiments/phase_aw_native_gicp_fusion'))
from runtime_journal import Journal,journal_json
class JournalTests(unittest.TestCase):
 def test_external_stream_roundtrip_and_bounded_objects(self):
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'journal';j=Journal(path)
   for k in range(200):
    row=dict(index=k,values=[k]*10);j.append(row)
    with path.open('a') as f:f.write(json.dumps(row)+'\n')
   self.assertEqual(len(j),200);self.assertEqual(j[0]['index'],0);self.assertEqual(j[-1]['index'],199);self.assertEqual(len(vars(j)),5);self.assertEqual(len(json.loads(journal_json(j))),200);self.assertEqual(sum(x['index'] for x in j),sum(range(200)))
 def test_auto_write_once_and_reject_random_access(self):
  with tempfile.TemporaryDirectory() as d:
   j=Journal(Path(d)/'wheel',True);j.append(dict(index=1));self.assertEqual(list(j),[dict(index=1)])
   with self.assertRaises(IndexError):_ = j[10]
