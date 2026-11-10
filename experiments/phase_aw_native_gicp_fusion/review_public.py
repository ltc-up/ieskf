"""Fail-closed current-tree release audit; invokes no commit or push."""
import argparse,ast,json,re,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--base',default='c647ac3224adee775d30dc6f9553c9b83612c060');a=p.parse_args();subprocess.run(['git','diff','--cached','--check'],check=True);files=subprocess.check_output(['git','diff',a.base,'--name-only'],text=True).splitlines();files+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],text=True).splitlines();files=sorted(set(files));unsafe=re.compile(r'/(?:home|media)/[^\s"\']+')
for name in files:
 f=Path(name);assert not name.startswith('data/'),'PRIVATE_INPUT_STAGED';assert f.suffix not in ('.db3','.pcd','.bin','.npz','.so','.npy','.bag'),'PRIVATE_OR_BINARY_PAYLOAD'
 if f.suffix=='.png':
  assert f.parent.name=='phase_aw_native_gicp_fusion';print('REVIEWED_AGGREGATE_FIGURE',name);continue
 text=f.read_text();assert not unsafe.search(text),'MACHINE_PATH:'+name;assert ('BEGIN'+' PRIVATE'+' KEY') not in text,'KEY_MATERIAL:'+name
 if f.suffix=='.json':json.loads(text)
 if f.suffix=='.py':ast.parse(text)
 print('REVIEWED',name)
print('PASS',len(files),'current-tree files; figures require separate visual review')
