import gzip,hashlib,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import archive_closed_log,repack_closed_log_xz
from log_io import open_text


class RepackTests(unittest.TestCase):
    def make(self,root):
        p=root/'run';p.mkdir();(p/'run_status.json').write_text('{"reason":"COMPLETED"}')
        source=p/'frames.jsonl';source.write_bytes(b'{"P":[0.123456789,2.1],"failure":"retained"}\n'*1000)
        payload=source.read_bytes()
        with patch.object(archive_closed_log,'ROOT',root):archive_closed_log.archive(source)
        return source,payload
    def test_exact_original_and_explicit_gzip_path_survive_repack(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,payload=self.make(root);gz=source.with_suffix('.jsonl.gz')
            with patch.object(repack_closed_log_xz,'ROOT',root):row=repack_closed_log_xz.repack(gz)
            self.assertFalse(gz.exists());self.assertEqual(row['original_sha256'],hashlib.sha256(payload).hexdigest())
            self.assertEqual(len(row['previous_archives']),1)
            for name in (source,gz,source.with_suffix('.jsonl.xz')):
                with open_text(name) as f:self.assertEqual(f.read().encode(),payload)
    def test_tampered_compressed_evidence_never_replaced(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,payload=self.make(root);gz=source.with_suffix('.jsonl.gz');gz.write_bytes(gz.read_bytes()+b'tamper')
            with patch.object(repack_closed_log_xz,'ROOT',root):
                with self.assertRaises(ValueError):repack_closed_log_xz.repack(gz)
            self.assertTrue(gz.exists());self.assertFalse(source.with_suffix('.jsonl.xz').exists())

if __name__=='__main__':unittest.main()
