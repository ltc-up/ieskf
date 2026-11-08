import unittest
import numpy as np
from ndt_global.integrity import directional,MapSupport,cluster,classify

class IntegrityTests(unittest.TestCase):
    def planes(self,normals):
        p=[];n=[]
        for normal in normals:
            normal=np.array(normal,float);u=np.cross(normal,np.eye(3)[np.argmin(abs(normal))]);v=np.cross(normal,u)
            for a in range(-2,3):
                for b in range(-2,3):p.append(a*u+b*v);n.append(normal)
        return np.array(p),np.array(n)
    def test_planes_and_guardrail(self):
        for ns,rank in [([[0,0,1]],3),([[0,0,1],[0,1,0]],5),([[0,0,1],[0,1,0],[1,0,0]],6),([[0,1,0]],3)]:
            p,n=self.planes(ns);self.assertEqual(directional(p,n)['rank'],rank)
    def test_corridor_and_missing_blocks(self):
        p,n=self.planes([[0,0,1],[0,1,0]])
        H=directional(p,n);self.assertEqual(H['rank'],5)
        # Repetition cannot add missing longitudinal normal information.
        self.assertEqual(directional(np.tile(p,(3,1)),np.tile(n,(3,1)))['rank'],5)
        self.assertEqual(directional(p[:25],n[:25])['rank'],3)
    def test_length_units_and_duplicates(self):
        p,n=self.planes([[0,0,1],[0,1,0],[1,0,0]])
        a=directional(p,n,10);b=directional(p*100,n,1000)
        np.testing.assert_allclose(a['eigenvalues'],b['eigenvalues'],atol=1e-14)
        np.testing.assert_allclose(a['eigenvalues'],directional(np.tile(p,(2,1)),np.tile(n,(2,1)),10)['eigenvalues'])
    def test_support_empty_and_absent_map(self):
        p,n=self.planes([[0,0,1]])
        m=MapSupport(p);self.assertEqual(m.diagnose(np.empty((0,3)),np.eye(4))['classification'],'NO_CLOUD')
        T=np.eye(4);T[2,3]=100
        self.assertEqual(m.diagnose(p,T)['classification'],'MAP_SUPPORT_MISSING')
    def test_clustering_multimode_and_chain(self):
        def c(x):
            T=np.eye(4);T[0,3]=x;return dict(pose=T,converged=True,fitness=.01)
        a=cluster([c(0),c(.2),c(5)])
        self.assertEqual(a['classification'],'AMBIGUOUS');self.assertEqual(a['independent_observations'],0)
        self.assertEqual(len(cluster([c(0),c(.8),c(1.6)])['clusters']),2)
    def test_single_peak_not_unique(self):
        c=dict(pose=np.eye(4),converged=True);m=cluster([c]);s=dict(classification='VALID_CANDIDATE',rank=6)
        self.assertEqual(classify(converged=True,support=s,modes=m),'UNKNOWN')
        self.assertEqual(classify(converged=True,support=s,modes={'classification':'AMBIGUOUS'}),'AMBIGUOUS')
        self.assertEqual(classify(converged=False),'NOT_CONVERGED')
        self.assertEqual(classify(received=False),'NO_CLOUD')
if __name__=='__main__':unittest.main()
