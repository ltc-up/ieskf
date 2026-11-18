"""Bounded Python-object retention; immutable original JSONL remains the journal."""
import json
class Journal:
 def __init__(self,path,auto_write=False):self.path=path;self.n=0;self.last=None;self.first=None;self.auto_write=auto_write
 def append(self,row):
  self.n+=1;self.last=row
  if self.first is None:self.first=row
  if self.auto_write:
   with self.path.open('a') as f:f.write(json.dumps(row)+'\n')
 def __len__(self):return self.n
 def __getitem__(self,i):
  if i==-1:return self.last
  if i==0:return self.first
  raise IndexError('JOURNAL_NOT_RANDOM_ACCESS')
 def __iter__(self):
  with self.path.open() as f:
   for line in f:yield json.loads(line)
def journal_json(value):
 if isinstance(value,Journal):
  with value.path.open() as stream:return '['+','.join(line.strip() for line in stream)+']'
 return json.dumps(value)
