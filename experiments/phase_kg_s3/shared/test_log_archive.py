import hashlib,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import archive_closed_log
from log_io import open_text

class ClosedLogArchiveTests(unittest.TestCase):
    def test_exact_roundtrip_manifest_and_original_path_reader(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=root/'run';run.mkdir()
            (run/'run_status.json').write_text(json.dumps(dict(reason='COMPLETED')))
            path=run/'frames.jsonl';payload=(json.dumps(dict(t=1763205554863639180,P=[.1]*225,label='真实记录'),ensure_ascii=False)+'\n').encode()*50
            path.write_bytes(payload)
            with patch.object(archive_closed_log,'ROOT',root):row=archive_closed_log.archive(path)
            self.assertFalse(path.exists());self.assertEqual(row['original_sha256'],hashlib.sha256(payload).hexdigest())
            with open_text(path) as f:self.assertEqual(f.read().encode(),payload)
            self.assertEqual(json.loads((run/'lossless_log_archive.json').read_text())['files'][0],row)
    def test_incomplete_run_never_releases_original(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'frames.jsonl';path.write_bytes(b'{}\n')
            with patch.object(archive_closed_log,'ROOT',root):
                with self.assertRaises(ValueError):archive_closed_log.archive(path)
            self.assertEqual(path.read_bytes(),b'{}\n');self.assertFalse((root/'frames.jsonl.gz').exists())
    def test_symlink_is_not_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);real=root/'real.jsonl';real.write_bytes(b'{}\n');link=root/'link.jsonl';link.symlink_to(real)
            with patch.object(archive_closed_log,'ROOT',root):
                with self.assertRaises(ValueError):archive_closed_log.archive(link)
            self.assertTrue(real.exists())

if __name__=='__main__':unittest.main()
