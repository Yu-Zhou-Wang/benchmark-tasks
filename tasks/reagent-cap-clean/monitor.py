"""Read-only contact/pose scoring; violations latch until the next reset."""
import importlib.abc,importlib.util,sys
import numpy as np
from specification import *
M={}
VIEW=None
POSE=None
ERROR=None
ACTIVE=False
FILTERS=[]

def np_(x):
    x=getattr(x,'torch',x)
    return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x.numpy() if hasattr(x,'numpy') else x)

def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
      [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def protected(p):
    r=np.linalg.norm(p[:2]);z=p[2]
    return (r<=.0241 and -.001<=z<=.0246) or (r<=.0305 and -.001<=z<=.001)

def reset(env):
    global ACTIVE
    ACTIVE=True
    M.clear();M.update(score=0.,contaminated=False,first_contact=None,removed=False,hold=0.,polls=0,points=0,fallen=False,error=ERROR,anchor=None)

def poll(mgr):
    global VIEW,POSE,ERROR,FILTERS
    try:
        if VIEW is None:
            from isaaclab.sim.utils.stage import get_current_stage
            from pxr import UsdPhysics
            stage=get_current_stage();cap=[];other=set()
            for prim in stage.Traverse():
                path=str(prim.GetPath())
                if '/env_0/' not in path:continue
                if prim.HasAPI(UsdPhysics.RigidBodyAPI) and ('/cap/' in path or path.endswith('/cap')):cap.append(path)
                if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
                if '/cap/' in path or '/bottle/' in path or path.endswith(('/cap','/bottle')):continue
                owner=prim
                while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
                other.add(str(owner.GetPath()) if owner else path)
            if len(cap)!=1:return
            sv=mgr.get_physics_sim_view()
            if sv is None:return
            FILTERS=sorted(other)
            VIEW=sv.create_rigid_contact_view(cap,filter_patterns=[FILTERS],max_contact_data_count=4096)
            POSE=sv.create_rigid_body_view(cap)
            print('cap monitor ready',cap,'filters',FILTERS,flush=True)
        if not ACTIVE:return
        # Physical contact pairs exclude only the bottle's initially clean neck.
        forces,points,normals,sep,counts,starts=VIEW.get_contact_data(dt=mgr.get_physics_dt())
        counts=np_(counts).reshape(-1);starts=np_(starts).reshape(-1)
        M['polls']+=1
        if not counts.any():return
        pose=np_(POSE.get_transforms())[0];R=rotation(pose[3:7]);p0=pose[:3]
        pts=np_(points);seps=np_(sep).reshape(-1)
        for other,(start,count) in zip(FILTERS,zip(starts,counts)):
            for i in range(int(start),int(start+count)):
                if seps[i]>.0005:continue
                p=(pts[i]-p0)@R;M['points']+=1
                if '/LeftArm/' in other or '/RightArm/' in other:M['last_robot_poll']=M['polls']
                if protected(p):
                    M['contaminated']=True
                    if M['first_contact'] is None:M['first_contact']={'local_point':p.tolist(),'separation':float(seps[i]),'other':other,'local_normal':(np_(normals)[i]@R).tolist()}
    except Exception as e:
        ERROR=repr(e)
        if M:M['error']=ERROR

def evaluate(s,clear,dt=.05):
    """All credit is read from measurements, never from controller phase."""
    values=np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]
    if not np.isfinite(values).all() or abs(np.linalg.norm(s.quat_xyzw)-1)>.01:
        M['error']='invalid pose/velocity';M['score']=0.;return
    R=rotation(s.quat_xyzw)
    center=s.pos+R@np.array([0,0,.0135])
    low=center[2]-(.0135*abs(R[2,2])+.029*np.sqrt(max(0,1-R[2,2]**2)))
    if low<TOP-.01 or not (-.19<=center[0]<=.91 and -.65<=center[1]<=.65):M['fallen']=True
    if low>=TOP+BOTTLE_HEIGHT+.02:M['removed']=True
    up=-R[2,2]>=np.cos(np.radians(15))
    placed=np.linalg.norm(center[:2]-PAD_XY)<=.02 and abs(low-PAD_TOP)<=.003
    slow=np.linalg.norm(s.lin_vel)<.01 and np.linalg.norm(s.ang_vel)<.2
    valid=up and placed and slow and clear
    if valid:
        if M['anchor'] is not None:
            a=M['anchor'];angle=np.arccos(np.clip((np.trace(np.array(a['rotation']).T@R)-1)/2,-1,1))
            if np.linalg.norm(center-a['center'])>.002 or angle>np.radians(3):
                M['hold']=0.;M['anchor']=None
        if M['anchor'] is None:M['anchor']={'center':center.tolist(),'rotation':R.tolist()}
        M['hold']+=dt
    else:M['hold']=0.;M['anchor']=None
    M['score']=0. if M['contaminated'] or M['fallen'] or M['error'] or M['polls']==0 else (1. if M['removed'] and M['hold']>=2-1e-8 else .25 if M['removed'] else 0.)
    M.update(center=center.tolist(),low=float(low),up=bool(up),placed=bool(placed),clear=bool(clear))

def step(env):
    from yam_arena.scene import object_states
    from scenekit import robot_links
    s=object_states(env,['cap'])['cap']
    center=s.pos+rotation(s.quat_xyzw)@np.array([0,0,.0135])
    links=robot_links(env)
    clear=M.get('last_robot_poll',-100)<M['polls']-12 and all(np.linalg.norm(p-center)>.070 for arm in links.values() for name,p in arm.items() if name in ('tip_left','tip_right'))
    evaluate(s,clear,env.unwrapped.step_dt)

NAME='isaaclab_physx.physics.physx_manager'
def patch(mod):
    cls=mod.PhysxManager;cfg=cls._configure_physics.__func__;original=cls.step.__func__
    def config(c):
        cfg(c)
        import carb
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
    def step(c):original(c);poll(c)
    cls._configure_physics=classmethod(config);cls.step=classmethod(step)
class Finder(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path,target=None):
        if fullname!=NAME:return None
        sys.meta_path.remove(self)
        try:spec=importlib.util.find_spec(fullname)
        finally:sys.meta_path.insert(0,self)
        old=spec.loader.exec_module
        def run(m):old(m);patch(m)
        spec.loader.exec_module=run;return spec
if NAME in sys.modules:patch(sys.modules[NAME])
else:sys.meta_path.insert(0,Finder())
