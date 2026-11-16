"""SI geometry for a bench-mounted door with a full-size chain latch."""
import math
import numpy as np

HINGE = np.array([0.42, 0.18, 0.7475])
WIDTH, HEIGHT, THICKNESS = 0.36, 0.31, 0.018
INITIAL_ANGLE = math.radians(-21.0)
SPRING_TARGET = math.radians(-35.0)
SPRING_K, SPRING_DAMPING = 0.60, 0.16
ANCHOR = np.array([-0.021, -0.393, 0.245])
HEAD = np.array([-0.013, -0.325, 0.245])
HEAD_HALF = np.array([0.002, 0.008, 0.009])
TAB = np.array([-0.026, 0., 0.])  # relative to head
ATTACH = np.array([-0.030, -0.014, 0.])
HANDLE = np.array([-0.055, -0.155, 0.130])
EXIT_Y = -0.260
PLATE_FRONT_X = -0.020
N_LINKS = 12
WIRE_R = 0.0016
CHAIN_SAG = 0.004

def rz(q):
    c,s=math.cos(q),math.sin(q)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def point(p, q):
    return HINGE + rz(q) @ np.asarray(p)

def rotation(quat):
    x,y,z,w=quat
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def chain_points():
    # Route outside both corners of the free edge. A straight chord would
    # intersect the plywood when the door is partly open.
    r=rz(INITIAL_ANGLE)
    control=np.array([r.T@ANCHOR,[.020,-.378,.245],[-.022,-.378,.245],HEAD+ATTACH])
    lengths=np.linalg.norm(np.diff(control,axis=0),axis=1)
    cumulative=np.r_[0,np.cumsum(lengths)]
    dist=np.linspace(0,cumulative[-1],N_LINKS+1)
    pts=np.stack([np.interp(dist,cumulative,control[:,i]) for i in range(3)],1)
    pts[:,2]-=CHAIN_SAG*np.sin(np.linspace(0,math.pi,N_LINKS+1))
    return pts@r.T

CHAIN_LENGTH=float(np.linalg.norm(np.diff(chain_points(),axis=0),axis=1).sum())
