"""Private-input services; output files must remain on the authorized mounted volume."""
import json,subprocess,time,struct,hashlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial import cKDTree

def read_cloud(path, timeout_s=.1):
    # Legacy deskew adapter emits DONE just before the local ofstream destructor.
    # Wait for its declared byte count, rather than consuming a partial buffer.
    deadline=time.perf_counter()+timeout_s
    while True:
        with open(path,'rb') as f:
            header=f.read(8)
            if len(header)==8:
                n=struct.unpack('<Q',header)[0];v=np.fromfile(f,dtype='<f4')
                if n<=100_000_000 and len(v)==3*n:return v.reshape(-1,3)
        if time.perf_counter()>=deadline:raise ValueError('TRUNCATED_CLOUD')
        time.sleep(.0001)
def write_cloud(path,v):
    v=np.asarray(v,'<f4')
    with open(path,'wb') as f:f.write(struct.pack('<Q',len(v)));v.tofile(f)
def partition(points):
    cell=np.floor(np.asarray(points)/2.).astype(np.int64)
    h=cell[:,0]*73856093 ^ cell[:,1]*19349663 ^ cell[:,2]*83492791
    hold=(h%5)==0
    return points[~hold],points[hold]
class Service:
    def __init__(self,binary,log,args=()):
        self.proc=subprocess.Popen([str(binary),*map(str,args)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=open(log,'w'),text=True)
    def call(self,request):
        t=time.perf_counter_ns();self.proc.stdin.write(json.dumps(request)+'\n');self.proc.stdin.flush();line=self.proc.stdout.readline()
        if not line:raise RuntimeError('MATCHER_SERVICE_TERMINATED')
        return json.loads(line),(time.perf_counter_ns()-t)*1e-6
    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=30)
class Validator:
    def __init__(self,target):self.target=read_cloud(target).astype(float);self.tree=cKDTree(self.target)
    def cost(self,points,T):
        p=points[np.linspace(0,len(points)-1,min(len(points),1024),dtype=int)] if len(points) else np.empty((0,3))
        if not len(p):return dict(cost=4.,coverage=0.,samples=0)
        T=np.asarray(T);q=p@T[:3,:3].T+T[:3,3];d,_=self.tree.query(q,workers=1)
        # Bounded generalized evidence, not backend fitness or calibrated probability.
        return dict(cost=float(np.mean(np.minimum(d,2.)**2)/.25),coverage=float(np.mean(d<1.)),samples=len(p))
def hashfile(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(4<<20),b''):h.update(b)
    return h.hexdigest()
