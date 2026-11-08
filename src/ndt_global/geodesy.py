"""Original WGS84 coordinate math; caller owns datum/height/antenna qualification."""
import numpy as np

def ecef(lla):
    lat,lon,h=np.asarray(lla,float);lat,lon=np.deg2rad([lat,lon])
    if not np.isfinite([lat,lon,h]).all() or abs(lat)>np.pi/2:raise ValueError('INVALID_LLA')
    a=6378137.;f=1/298.257223563;e2=f*(2-f);N=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.array([(N+h)*np.cos(lat)*np.cos(lon),(N+h)*np.cos(lat)*np.sin(lon),(N*(1-e2)+h)*np.sin(lat)])

def enu(lla,origin):
    lat,lon=np.deg2rad(np.asarray(origin)[:2]);s,c=np.sin,np.cos
    R=np.array([[-s(lon),c(lon),0],[-s(lat)*c(lon),-s(lat)*s(lon),c(lat)],[c(lat)*c(lon),c(lat)*s(lon),s(lat)]])
    return R@(ecef(lla)-ecef(origin))

def enu_from_ned(v):return np.asarray(v)[[1,0,2]]*np.array([1,1,-1])
