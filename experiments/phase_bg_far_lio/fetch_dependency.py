"""Approved software archives only; cumulative 512 MiB budget, no dataset URLs."""
import argparse, hashlib, json, tarfile, urllib.request
from pathlib import Path
LIMIT=512*1024**2
PREFIX="https://codeload.github.com/"
def fetch(root,name,url):
 root=Path(root); ledger=root/"download_ledger.json"
 rows=json.loads(ledger.read_text()) if ledger.exists() else []
 if not url.startswith(PREFIX): raise ValueError("Software archive host not allowed")
 if any(r["name"]==name for r in rows): raise ValueError("Already recorded; reuse existing dependency")
 if any(root.glob(name+"*")): raise ValueError("Existing output; inspect failed archive rather than overwrite")
 used=sum(r["bytes"] for r in rows); dst=root/(name+".tar.gz"); n=0;sha=hashlib.sha256()
 row=dict(name=name,url=url,bytes=0,status="STARTED");rows.append(row)
 def save(): ledger.write_text(json.dumps(rows,indent=2)+"\n")
 save()
 try:
  with urllib.request.urlopen(url,timeout=40) as response,dst.open("xb") as f:
   while True:
    data=response.read(min(1024*1024,LIMIT-used-n+1))
    if not data: break
    n+=len(data);row["bytes"]=n;save()
    if used+n>LIMIT: raise RuntimeError("Download budget exceeded; stop")
    f.write(data);sha.update(data)
  row["sha256"]=sha.hexdigest();row["status"]="DOWNLOADED";save()
  output=root/name;output.mkdir()
  with tarfile.open(dst) as t:t.extractall(output,filter="data")
  row["status"]="EXTRACTED";save()
 except Exception as e:
  row["status"]="FAILED";row["error"]=str(e);save();raise
 print(name,n,"bytes; cumulative",used+n)
if __name__=="__main__":
 p=argparse.ArgumentParser();p.add_argument("root");p.add_argument("name");p.add_argument("url");a=p.parse_args();fetch(a.root,a.name,a.url)
