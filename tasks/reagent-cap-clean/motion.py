"""Robot-only Cartesian path: quarter-turn, exterior handover, quarter-turn, park."""
from dataclasses import dataclass
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix
from specification import TOP,BOTTLE_XY,CAP_START_Z,PAD_XY,PAD_TOP

def ry(a):return axis_angle_matrix(np.array([0.,1.,0.]),a)
def rz(a):return axis_angle_matrix(np.array([0.,0.,1.]),a)
D=grip_rotation();SIDE=rz(np.pi/2)@ry(-np.pi/2)@D
GRIP=-.025

@dataclass
class W:
    duration:float
    pos:tuple
    rot:np.ndarray
    finger:float
    label:str

class Plan:
    def __init__(self,side,waypoints):
        self.base=np.array(station.BASES[side]);self.q=np.array(READY,dtype=float)
        self.p,self.r=ik.tcp_pose(self.q,self.base);self.ws=waypoints
        self.worst=np.zeros(2)
    def action(self,t):
        p,r,f=self.p,self.r,FINGER_OPEN
        for w in self.ws:
            if t<w.duration:
                s=ik.smoothstep(t/w.duration)
                p=p+s*(np.array(w.pos)-p);r=ik.slerp_matrix(r,w.rot,s);f=f+s*(w.finger-f);label=w.label
                break
            t-=w.duration;p=np.array(w.pos);r=w.rot;f=w.finger
        else:label='done'
        self.q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),self.q,base_pos=self.base,iters=100)
        self.worst=np.maximum(self.worst,[pe,np.degrees(re)])
        return np.r_[self.q,float(np.clip(f/FINGER_OPEN,0,1))],label

def plans():
    x,y=BOTTLE_XY;cz=CAP_START_Z+.0135
    h=np.array([.32,.04,TOP+.16]);left_h=h+np.array([0,0,.006]);left_side=h+np.array([-.006,0,0])
    rp=h+np.array([0,-.004,0]);away=rp+np.array([0,-.105,0]);target=np.array([*PAD_XY,PAD_TOP+.0135-.006])+np.array([0,-.004,0])
    l=[W(2,(x,y,TOP+.16),D,FINGER_OPEN,'approach'),W(2,(x,y,cz+.006),D,FINGER_OPEN,'descend'),
       W(1.2,(x,y,cz+.006),D,GRIP,'grasp exterior'),W(2,(x,y,TOP+.16),D,GRIP,'lift'),
       W(2,tuple(left_h),D,GRIP,'transfer'),W(3,tuple(left_side),ry(-np.pi/2)@D,GRIP,'quarter turn'),
       W(6,tuple(left_side),ry(-np.pi/2)@D,GRIP,'wait for receiving grasp'),
       W(1.2,tuple(left_side),ry(-np.pi/2)@D,FINGER_OPEN,'release to receiver'),
       W(2,tuple(left_side+[-.13,.10,.02]),ry(-np.pi/2)@D,FINGER_OPEN,'withdraw'),
       W(15,(.20,.27,TOP+.16),D,FINGER_OPEN,'park')]
    r=[W(10,tuple(away),ry(np.pi/2)@SIDE,FINGER_OPEN,'wait clear'),
       W(3,tuple(rp),ry(np.pi/2)@SIDE,FINGER_OPEN,'approach exterior across other diameter'),
       W(2,tuple(rp),ry(np.pi/2)@SIDE,GRIP,'receive'),W(6.4,tuple(rp),ry(np.pi/2)@SIDE,GRIP,'hold while first arm leaves'),
       W(3,tuple(rp),SIDE,GRIP,'turn hollow side upward'),
       W(2,(target[0],target[1],TOP+.16),SIDE,GRIP,'carry'),
       W(2,tuple(target),SIDE,GRIP,'lower'),W(1,tuple(target),SIDE,GRIP,'seat on rest'),W(1.5,tuple(target),SIDE,FINGER_OPEN,'release'),
       W(2,tuple(target+[0,-.07,.025]),SIDE,FINGER_OPEN,'withdraw'),W(3,tuple(target+[0,-.07,.025]),SIDE,FINGER_OPEN,'observe')]
    return {s:Plan(s,w) for s,w in [('left',l),('right',r)]}

if __name__=='__main__':
    ps=plans()
    for t in np.arange(.05,38,.05):
        for p in ps.values():p.action(t)
    print({s:p.worst.tolist() for s,p in ps.items()})
