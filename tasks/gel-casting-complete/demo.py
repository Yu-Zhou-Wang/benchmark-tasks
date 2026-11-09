"""Robot-only Cartesian controller for the gel caster; never writes object poses."""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
HZ=20
def encode(value,**kwargs):
    return json.dumps(value,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist(),**kwargs)

class Hand:
    def __init__(self,side):
        from yam_arena import ik,station
        from yam_arena.constants import READY
        self.side=side;self.base=np.asarray(station.BASES[side]);self.q=np.array(READY,float);self.grip=1.
        self.pos,self.rot=ik.tcp_pose(self.q,self.base)
        self.home=(self.pos.copy(),self.rot.copy());self.max_error=0.
    def aim(self,pos,rot):
        from yam_arena import ik
        self.q,err,_=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),q_init=self.q,base_pos=self.base,iters=100)
        self.pos=np.asarray(pos);self.rot=np.asarray(rot);self.max_error=max(self.max_error,float(err))
    def move(self,pos,rot=None,seconds=1.,grip=None):
        from yam_arena import ik
        rot=self.rot if rot is None else rot
        p0,r0,g0=self.pos.copy(),self.rot.copy(),self.grip
        g1=g0 if grip is None else grip
        for i in range(1,max(1,int(seconds*HZ))+1):
            t=ik.smoothstep(i/max(1,int(seconds*HZ)))
            self.grip=g0+(g1-g0)*t
            self.aim(p0+t*(np.asarray(pos)-p0),ik.slerp_matrix(r0,rot,t));yield
    def close(self):yield from self.move(self.pos,seconds=.7,grip=0.)
    def release(self):yield from self.move(self.pos,seconds=.6,grip=1.)
    def homeward(self):yield from self.move(self.home[0],self.home[1],2.,grip=1.)
    def command(self):return np.r_[self.q,self.grip]
def pause(seconds):
    for _ in range(round(HZ*seconds)):yield
def parallel(*plans):
    active=list(map(iter,plans))
    while active:
        for plan in active.copy():
            try:next(plan)
            except StopIteration:active.remove(plan)
        if active:yield
class Controller:
    def __init__(self,emb):
        self.emb=emb;self.task=emb.task.module;self.left=Hand('left');self.right=Hand('right');self.phase='start';self.failure=None
    def states(self):
        from yam_arena.scene import object_states
        return object_states(self.emb._env,self.emb.task.names)
    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.left.command(),self.right.command()))

def close_cam(ctx,side):
    from yam_arena.demo import grip_rotation
    t=ctx.task
    operating=ctx.left if side=='left' else ctx.right
    sign=-1 if side=='left' else 1
    frame=ctx.states()['frame']
    key=side+'_hinge';start=frame.joints[key]
    pivot=np.array([.009,sign*.054,.0495])
    # The visible cradle braces the feet. Finger collision-mesh support varies
    # with the wing angle even though the wrist orientation stays downward.
    def waypoint(angle,frm,clearance=0.):
        c,s=math.cos(angle),math.sin(angle)
        support=.00774*abs(c)+.00395*abs(s)
        arm_offset=np.array([.00325+support-.0007+clearance,-sign*.020,.033])
        turn=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        p=t.point(frm,pivot+turn@arm_offset)
        return p,grip_rotation(0.)
    pick,ori=waypoint(start,frame,.004)
    ctx.phase=f'contact_{side}_cam'
    yield from operating.move(pick+[0,0,.065],ori,1.8,grip=0.)
    yield from operating.move(pick,ori,1.5)
    pick,ori=waypoint(start,ctx.states()['frame'])
    yield from operating.move(pick,ori,1.)
    ctx.phase=f'rotate_{side}_cam'
    target=math.radians(-30 if side=='left' else 30)
    for k in range(1,101):
        measured=ctx.states()['frame']
        if abs(measured.joints[key])<math.radians(45):break
        p,r=waypoint(start+(target-start)*k/100,measured)
        operating.aim(p,r);yield
    yield from pause(.3)
    # Unload the cam contact horizontally before rising past the frame post.
    frame=ctx.states()['frame'];angle=frame.joints[key]
    normal=t.rotation(frame.quat_xyzw)@np.array([math.cos(angle),math.sin(angle),0.])
    ctx.phase=f'retract_{side}_cam'
    yield from operating.move(operating.pos+.020*normal,seconds=.8)
    yield from operating.move(operating.pos+[0,0,.07],seconds=1.5)
    yield from operating.homeward()

def settle_frame(ctx,hand,position,rotation,seconds=2.):
    """Small visual feedback corrections through real joint commands only."""
    from yam_arena import ik
    for _ in range(round(HZ*seconds)):
        frame=ctx.states()['frame']
        delta=.25*(np.asarray(position)-frame.pos)
        length=np.linalg.norm(delta)
        if length>.0015:delta*=.0015/length
        correction=rotation@ctx.task.rotation(frame.quat_xyzw).T
        turn=ik.slerp_matrix(np.eye(3),correction,.15)
        hand.aim(hand.pos+delta,turn@hand.rot)
        yield

def release_frontward(hand):
    """Keep the rear jaw clear of the stand while the front jaw opens."""
    from yam_arena import ik
    yield from hand.move(hand.pos,seconds=.7,grip=.020/.0939)
    start=hand.pos.copy();rotation=hand.rot.copy()
    for k in range(1,31):
        aperture=.020+(.0939-.020)*ik.smoothstep(k/30)
        hand.grip=aperture/.0939
        hand.aim(start-.5*(aperture-.020)*rotation[:,0],rotation)
        yield

def dock_forward_offset(height):
    """Clear the open pressure shoe, then align before the low foot guides."""
    blend=float(np.clip((height-.008)/(.020-.008),0.,1.))
    return .002*blend*blend*(3.-2.*blend)

def dock(ctx):
    from yam_arena import ik
    from yam_arena.demo import grip_rotation
    t=ctx.task;carry=ctx.right;press=ctx.left
    frame=ctx.states()['frame']
    grasp_local=np.array([-.001,.0555,.076])
    pick=t.point(frame,grasp_local)
    tilt=math.radians(30);c,s=math.cos(tilt),math.sin(tilt)
    down=np.array([[1.,0.,0.],[0.,-c,s],[0.,-s,-c]])
    roll=np.array([[1.,0.,0.],[0.,c,s],[0.,-s,c]])
    back=math.radians(25);cb,sb=math.cos(back),math.sin(back)
    pitch=np.array([[cb,0.,sb],[0.,1.,0.],[-sb,0.,cb]])
    press_rotation=pitch@roll@grip_rotation(0.)
    tip_offset=-.01284*press_rotation[:,2]
    ctx.phase='pick_cassette'
    yield from carry.move(pick+[0,0,.065],down,1.8,grip=1.)
    yield from carry.move(pick,down,1.4)
    yield from carry.close()
    yield from pause(.4)
    yield from carry.move(carry.pos+[0,0,.140],seconds=4.)
    if not ctx.task.STATE.get('transported') or not ctx.task.STATE.get('aligned'):
        ctx.failure='The physical grasp did not lift and retain an aligned cassette.'
        ctx.phase='stopped_after_pick';return
    ctx.phase='carry_to_stand'
    stand=ctx.states()['stand']
    above=t.point(stand,grasp_local+[.002,0,.140])
    yield from carry.move(above,down,3.)
    stand=ctx.states()['stand'];desired_rotation=t.rotation(stand.quat_xyzw)
    yield from settle_frame(ctx,carry,stand.pos+desired_rotation@np.array([.002,0,.140]),desired_rotation,3.)
    if not ctx.task.STATE.get('aligned'):
        ctx.failure='The plates lost their aligned seal during the physical transfer.'
        ctx.phase='stopped_after_transfer';return
    frame=ctx.states()['frame'];frame_rotation=t.rotation(frame.quat_xyzw)
    # Capture the measured grasp and its load compensation, rather than
    # assuming the original pick point remained perfectly centred in the pads.
    grasp_offset=frame_rotation.T@(carry.pos-frame.pos)
    grasp_rotation=frame_rotation.T@carry.rot
    # Press the back tab downward: the front pressure shoe rises against its
    # return spring. Keep that physical contact while the other arm inserts.
    tab_local=np.array([-.0285,0,.120])
    stand=ctx.states()['stand'];tab=t.point(stand,tab_local)
    ctx.phase='hold_top_clip_open'
    yield from press.move(tab+tip_offset+[0,0,.025],press_rotation,2.,grip=0.)
    yield from press.move(tab+tip_offset,seconds=1.3)
    pivot=np.array([-.025,0,.105]);taboff=np.array([-.0035,0,.015])
    for k in range(1,61):
        a=math.radians(-75)*k/60;c,s=math.cos(a),math.sin(a)
        turn=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        # Downward overtravel supplies the return-spring force through the
        # robot's finite joint stiffness; the lever still moves by contact only.
        p=t.point(ctx.states()['stand'],pivot+turn@taboff)+tip_offset+[0,0,-.012*k/60]
        press.aim(p,press_rotation);yield
    # Add limited physical pressing travel only when measured opening needs it.
    for _ in range(24):
        if ctx.task.STATE.get('clip_degrees',0.)<=-83.:break
        press.aim(press.pos+[0,0,-.00025],press_rotation);yield
    yield from pause(.4)
    if ctx.task.STATE.get('clip_degrees',0.)>-82.:
        ctx.failure='The physical finger contact did not hold the top lever open.'
        ctx.phase='stopped_at_top_clip';return
    ctx.phase='lower_onto_gasket'
    stand=ctx.states()['stand'];stand_rotation=t.rotation(stand.quat_xyzw)
    target=t.point(stand,grasp_offset)
    start_local=stand_rotation.T@(carry.pos-target)
    start_rotation=carry.rot.copy()
    for k in range(1,4*HZ+1):
        blend=ik.smoothstep(k/(4*HZ))
        height=start_local[2]+(.008-start_local[2])*blend
        # Preserve the measured start residual while following the verified
        # 2 mm frontward corridor; align fully by 8 mm above the seat.
        local=np.array([dock_forward_offset(height)+(start_local[0]-.002)*(1.-blend),
                        start_local[1]*(1.-blend),height])
        carry.aim(target+stand_rotation@local,
                  ik.slerp_matrix(start_rotation,stand_rotation@grasp_rotation,blend))
        yield
        if ctx.task.STATE.get('clip_degrees',0.)>-82.:
            ctx.failure='The top lever was no longer held open during insertion.'
            ctx.phase='stopped_during_insertion';return
    # Align while both frame feet are still above the guides. All subsequent
    # corrections start at the current wrist command, preserving the measured
    # grasp/load compensation instead of returning to an old grasp offset.
    ctx.phase='align_above_seat'
    stand=ctx.states()['stand'];stand_rotation=t.rotation(stand.quat_xyzw)
    for _ in settle_frame(ctx,carry,stand.pos+stand_rotation@np.array([0.,0.,.008]),stand_rotation,3.):
        yield
        if ctx.task.STATE.get('clip_degrees',0.)>-82.:
            ctx.failure='The top lever was no longer held open during pre-seat alignment.'
            ctx.phase='stopped_during_insertion';return
    ctx.phase='seat_cassette'
    measured=ctx.states();stand=measured['stand'];stand_rotation=t.rotation(stand.quat_xyzw)
    initial_height=float((stand_rotation.T@(measured['frame'].pos-stand.pos))[2])
    for k in range(1,3*HZ+1):
        height=initial_height*(1.-ik.smoothstep(k/(3*HZ)))
        stand=ctx.states()['stand'];stand_rotation=t.rotation(stand.quat_xyzw)
        yield from settle_frame(ctx,carry,stand.pos+stand_rotation@np.array([0.,0.,height]),
                                stand_rotation,1/HZ)
        if ctx.task.STATE.get('clip_degrees',0.)>-82.:
            ctx.failure='The top lever was no longer held open during seating.'
            ctx.phase='stopped_during_insertion';return
    stand=ctx.states()['stand'];stand_rotation=t.rotation(stand.quat_xyzw)
    for _ in settle_frame(ctx,carry,stand.pos,stand_rotation,2.):
        yield
        if ctx.task.STATE.get('clip_degrees',0.)>-82.:
            ctx.failure='The top lever was no longer held open during seating.'
            ctx.phase='stopped_during_insertion';return
    for _ in pause(.5):
        yield
        if ctx.task.STATE.get('clip_degrees',0.)>-82.:
            ctx.failure='The top lever was no longer held open before release.'
            ctx.phase='stopped_during_insertion';return
    if not t.geometry_metrics(ctx.states())['docked']:
        ctx.failure='The cassette was not physically seated and aligned before gripper release.'
        ctx.phase='stopped_before_release';return
    yield from release_frontward(carry)
    yield from carry.move(carry.pos+[0,0,.06],seconds=1.2)
    ctx.phase='release_top_clip'
    yield from press.move(press.pos+[0,0,.025],seconds=2.)
    yield from parallel(carry.homeward(),press.homeward())

def plan(ctx,variant):
    if variant=='idle':yield from pause(6.);return
    if variant=='docking':
        yield from dock(ctx)
        if not ctx.failure:ctx.phase='verify_release';yield from pause(4.)
        return
    yield from close_cam(ctx,'right')
    if variant=='right':yield from parallel(ctx.left.homeward(),ctx.right.homeward());return
    yield from close_cam(ctx,'left')
    if variant=='cams':yield from parallel(ctx.left.homeward(),ctx.right.homeward());return
    if not all(ctx.task.STATE.get('cams_closed',[])) or not ctx.task.STATE.get('aligned'):
        ctx.failure='Both cams must physically close around aligned glass before transfer.'
        ctx.phase='stopped_after_clamping';return
    yield from dock(ctx)
    if ctx.failure:return
    ctx.phase='verify_release'
    yield from pause(4.)

def run(args):
    import time
    run_started=time.monotonic()
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment,ARMS
    from yam_arena.constants import FINGER_JOINTS,FINGER_OPEN
    from source_fingerprint import capture
    from PIL import Image
    source_root=Path(__file__).resolve().parent
    source_before=capture(source_root)
    record_every=getattr(args,'record_every',1)
    if record_every<1:raise ValueError('record_every must be a positive integer')
    sparse_cameras=args.no_record or record_every>1
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    task_file='task_docking_fixture.py' if args.variant=='docking' else 'task.py'
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name(task_file)),object_jitter=0,
        control_hz=HZ,cam_width=args.width,cam_height=args.height)
    obs=emb.reset(Scene(id='gel-casting-demo',instruction=emb.task.module.INSTRUCTION),seed=None)
    reset_finished=time.monotonic()
    reset_configuration={'seed':None,'object_jitter_m':0.,'scene_id':'gel-casting-demo',
        'scene':emb.task.scene,'task_file':task_file,'instruction':emb.task.module.INSTRUCTION,
        'control_hz':HZ,'physics_dt_s':float(emb._base.sim.get_physics_dt())}
    if args.close_view:
        emb._base.scene['top_cam'].set_world_poses_from_view(eyes=[[.65,-.40,1.1]],targets=[[.25,0,.80]])
        for _ in range(4):emb._base.sim.render()
        obs=emb._observation()
    original_observation=emb._observation
    if sparse_cameras:
        from inspect_robots.types import Observation
        emb._base.render_enabled=False
        # Arena's camera_obs group reads sensor.data during every env.step;
        # those lazy reads render cameras even when Kit pumping is disabled.
        # Preserve joint observations while leaving camera reads to explicit
        # sampled recording, snapshots, and the final observation below.
        observation_manager=emb._base.observation_manager
        def state_only_observations(update_history=False):
            observations={'policy':observation_manager.compute_group('policy',update_history=update_history)}
            observation_manager._obs_buffer=observations
            return observations
        observation_manager.compute=state_only_observations
        emb._observation=lambda:Observation(images={},state={},instruction=emb._instruction)
    ctx=Controller(emb); script=iter(plan(ctx,args.variant))
    # These are measurements, never action corrections. Preserve raw values so
    # contact/solver excursions cannot disappear behind the observation clamp.
    # The 62.800169 um offset is the audited collision-mesh inner-face gap at
    # the TCP plane with both prismatic finger joints at zero. Other axial
    # sections of the tapered fingers have different clearances.
    closed_pad_gap_at_tcp_m=0.00006280016929622887
    gap_threshold_m=.094
    gripper_trace={'sample_hz':HZ,'sampling':'after_each_control_step_plus_initial_reset',
        'joint_names':list(FINGER_JOINTS),'joint_units':'metres',
        'nominal_gap_definition':'-(joint7 + joint8)',
        'pad_gap_at_tcp_definition':'nominal_gap_m + closed_pad_gap_at_tcp_m',
        'closed_pad_gap_at_tcp_m':closed_pad_gap_at_tcp_m,
        'commanded_max_nominal_gap_m':-2*FINGER_OPEN,
        'gap_threshold_m':gap_threshold_m,'joint_limits_m':{},'samples':[]}
    gripper_sources=[]
    for side,name in zip(('left','right'),ARMS):
        asset=emb._base.scene[name];ids=emb._ids[name][1]
        gripper_sources.append((side,asset,ids))
        gripper_trace['joint_limits_m'][side]=asset.data.joint_pos_limits.torch[0,ids].detach().cpu().tolist()
    def measure_grippers(step):
        raw=emb._torch.cat([asset.data.joint_pos.torch[0,ids]
                           for _,asset,ids in gripper_sources]).detach().cpu().tolist()
        hands={}
        for index,(side,_,_) in enumerate(gripper_sources):
            q=raw[2*index:2*index+2]
            command=float(getattr(ctx,side).grip)
            nominal_gap=-sum(q)
            hands[side]={'command_normalized':command,
                         'command_nominal_gap_m':-2*FINGER_OPEN*command,
                         'finger_positions_m':dict(zip(FINGER_JOINTS,q)),
                         'measured_normalized_unclamped':sum(q)/(2*FINGER_OPEN),
                         'nominal_gap_m':nominal_gap,
                         'pad_gap_at_tcp_m':nominal_gap+closed_pad_gap_at_tcp_m}
        sample={'step':step,'time_s':(step+1)/HZ,'phase':ctx.phase,'hands':hands}
        gripper_trace['samples'].append(sample)
        return hands
    measured_grippers=measure_grippers(-1)
    physics_trace=None
    if getattr(args,'physics_trace',False):
        base=emb._base
        physics_dt=float(base.sim.get_physics_dt())
        body_names=('frame','rear','front','stand')
        joint_names={name:list(base.scene[name].joint_names) for name in ('frame','stand')}
        physics_trace={'physics_dt':physics_dt,'phase':'release_top_clip_and_verify_release',
                       'pose_format':['x','y','z','qx','qy','qz','qw'],
                       'velocity_format':['vx','vy','vz','wx','wy','wz'],'samples':[]}
        original_scene_update=base.scene.update
        first_trace_step=None;last_trace_step=None
        def traced_scene_update(*update_args,**update_kwargs):
            nonlocal first_trace_step,last_trace_step
            result=original_scene_update(*update_args,**update_kwargs)
            if ctx.phase not in ('release_top_clip','verify_release'):return result
            physics_step=int(base.sim.get_physics_step_count())
            if physics_step==last_trace_step:return result
            # Read the fresh physics buffers after their per-substep update.
            # One packed transfer avoids separate CPU synchronization per body.
            tensors=[]
            for name in body_names:
                data=base.scene[name].data
                tensors.extend((data.root_link_pose_w.torch[0],data.root_link_vel_w.torch[0]))
            for name in joint_names:tensors.append(base.scene[name].data.joint_pos.torch[0])
            values=emb._torch.cat(tensors).detach().cpu().tolist()
            bodies={};cursor=0
            for name in body_names:
                bodies[name]={'pose_w_xyzw':values[cursor:cursor+7],'velocity_w':values[cursor+7:cursor+13]}
                cursor+=13
            joints={}
            for name,names in joint_names.items():
                joints[name]=dict(zip(names,values[cursor:cursor+len(names)]));cursor+=len(names)
            if first_trace_step is None:first_trace_step=physics_step
            physics_trace['samples'].append({'physics_step':physics_step,
                'time_s':physics_step*physics_dt,'elapsed_s':(physics_step-first_trace_step)*physics_dt,
                'bodies':bodies,'joints':joints})
            last_trace_step=physics_step
            return result
        base.scene.update=traced_scene_update
    frames={};trace=[];last_observation=obs
    def record(obs):
        if args.no_record:return
        for camera,pixels in obs.images.items():frames.setdefault(camera,[]).append(np.asarray(pixels).copy())
    record(obs)
    if obs.images:
        top=next((k for k in obs.images if 'top' in k),next(iter(obs.images)))
        Image.fromarray(np.asarray(obs.images[top])).save(out/'setup.png')
    completed=None;success=False;info={};last_phase=None
    loop_started=time.monotonic()
    for step in range(args.max_steps):
        try:next(script)
        except StopIteration:
            if completed is None:completed=step
        result=emb.step(ctx.action());info=result.info
        measured_grippers=measure_grippers(step)
        if not args.no_record and (step+1)%record_every==0:
            last_observation=original_observation() if sparse_cameras else result.observation
            record(last_observation)
        if step%20==0:
            measured=ctx.states()
            row={'step':step,'phase':ctx.phase,'score':info.get('score'),**ctx.task.STATE.copy(),
                 'desired_tcps':{'left':ctx.left.pos.tolist(),'right':ctx.right.pos.tolist()},
                 'grippers':measured_grippers,
                 'body_motion':{n:{'position':o.pos.tolist(),'speed':float(np.linalg.norm(o.lin_vel)),
                                   'spin':float(np.linalg.norm(o.ang_vel))}
                                for n,o in measured.items() if n in ('frame','rear','front','stand')}}
            trace.append(row);print(encode(row),flush=True)
        if args.snapshots and (ctx.phase!=last_phase or step%40==0):
            for _ in range(3):emb._base.sim.render()
            snap=original_observation()
            Image.fromarray(np.asarray(snap.images['top_cam'])).save(out/f'{step:04d}_{ctx.phase}.png')
        last_phase=ctx.phase
        if result.terminated:
            success=result.termination_reason=='success';break
        if completed is not None and step-completed>(20 if ctx.failure else 80):break
    loop_finished=time.monotonic()
    timings={'startup_and_reset_s':reset_finished-run_started,
        'setup_after_reset_s':loop_started-reset_finished,
        'controller_loop_s':loop_finished-loop_started}
    gripper_summary={k:v for k,v in gripper_trace.items() if k!='samples'}
    gripper_summary['sample_count']=len(gripper_trace['samples'])
    gripper_summary['hands']={}
    for side,_,_ in gripper_sources:
        samples=[row['hands'][side] for row in gripper_trace['samples']]
        q=np.array([[s['finger_positions_m'][n] for n in FINGER_JOINTS] for s in samples])
        limits=np.asarray(gripper_trace['joint_limits_m'][side])
        gaps=np.array([s['pad_gap_at_tcp_m'] for s in samples])
        nominal_gaps=[s['nominal_gap_m'] for s in samples]
        commands=[s['command_normalized'] for s in samples]
        gripper_summary['hands'][side]={
            'command_normalized_range':[min(commands),max(commands)],
            'finger_position_ranges_m':{n:[float(q[:,i].min()),float(q[:,i].max())]
                                       for i,n in enumerate(FINGER_JOINTS)},
            'nominal_gap_range_m':[min(nominal_gaps),max(nominal_gaps)],
            'pad_gap_at_tcp_range_m':[float(gaps.min()),float(gaps.max())],
            'samples_above_94mm':int(np.count_nonzero(gaps>gap_threshold_m)),
            'max_94mm_exceedance_m':float(max(0.,gaps.max()-gap_threshold_m)),
            'max_joint_limit_exceedance_m':float(max(0.,np.max(limits[:,0]-q),np.max(q-limits[:,1]))),
            'final':samples[-1]}
    result={'success':success,'task_score':1. if success else info.get('score',0),'steps':step+1,
            'record_every':record_every,'video_fps':None if args.no_record else HZ/record_every,
            'variant':args.variant,'controller_failure':ctx.failure,'final_state':ctx.task.STATE,'max_ik_error_mm':
            {'left':ctx.left.max_error*1000,'right':ctx.right.max_error*1000},
            'gripper_telemetry':gripper_summary,'reset_configuration':reset_configuration,
            'timing_s':timings}
    source_after=capture(source_root)
    manifest={'schema_version':1,'variant':args.variant,'source_before':source_before,
              'source_after':source_after,'source_unchanged':source_before==source_after,
              'reset_configuration':reset_configuration,'timing_s':timings}
    result.update(run_manifest='run_manifest.json',source_unchanged=manifest['source_unchanged'])
    (out/'run_manifest.json').write_text(encode(manifest,indent=2))
    (out/'result.json').write_text(encode(result,indent=2))
    (out/'trace.json').write_text(encode(trace,indent=2))
    (out/'gripper_trace.json').write_text(encode(gripper_trace,indent=2))
    if physics_trace is not None:
        (out/'physics_trace.json').write_text(encode(physics_trace,indent=2))
    if sparse_cameras:
        emb._base.render_enabled=True
        for _ in range(4):emb._base.sim.render()
        last_observation=original_observation()
    video_started=time.monotonic()
    if frames:
        import imageio.v2 as imageio
        for camera,images in frames.items():
            imageio.mimsave(out/(camera+'.mp4'),images,fps=HZ/record_every,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
    timings['video_export_s']=time.monotonic()-video_started if frames else 0.
    timings['total_through_video_export_s']=time.monotonic()-run_started
    # An off-cadence terminal image is saved separately so the video retains
    # uniform timestamps while final snapshots show the exact terminal state.
    for camera,pixels in last_observation.images.items():Image.fromarray(np.asarray(pixels)).save(out/(camera+'_final.png'))
    # Save an additional view only after the trajectory video and terminal
    # views are complete. Camera rendering advances no physics or object state.
    closeup_started=time.monotonic()
    stand_origin=ctx.states()['stand'].pos
    closeup_eye=stand_origin+np.array([.28,-.22,.20])
    closeup_target=stand_origin+np.array([0.,0.,.055])
    camera=emb._base.scene['top_cam']
    physics_step_before=int(emb._base.sim.get_physics_step_count())
    camera.set_world_poses_from_view(eyes=[closeup_eye.tolist()],targets=[closeup_target.tolist()])
    for _ in range(4):emb._base.sim.render()
    camera.update(0.,force_recompute=True)
    pixels=camera.data.output['rgb']
    pixels=pixels.torch if hasattr(pixels,'torch') else pixels
    Image.fromarray(pixels[0,:,:,:3].detach().cpu().numpy()).save(out/'seating_closeup.png')
    result['seating_closeup']={'image':'seating_closeup.png','eye':closeup_eye.tolist(),
        'target':closeup_target.tolist(),'physics_step_before':physics_step_before,
        'physics_step_after':int(emb._base.sim.get_physics_step_count())}
    timings['closeup_capture_s']=time.monotonic()-closeup_started
    timings['total_through_closeup_s']=time.monotonic()-run_started
    (out/'run_manifest.json').write_text(encode(manifest,indent=2))
    (out/'result.json').write_text(encode(result,indent=2))
    print('RESULT',encode(result),flush=True)
    return result
if __name__=='__main__':
    def positive_int(value):
        number=int(value)
        if number<1:raise argparse.ArgumentTypeError('must be a positive integer')
        return number
    p=argparse.ArgumentParser();p.add_argument('--out',default='runs/demo');p.add_argument('--variant',choices=['full','cams','right','idle','docking'],default='full')
    p.add_argument('--max-steps',type=int,default=2400);p.add_argument('--no-record',action='store_true')
    p.add_argument('--record-every',type=positive_int,default=1)
    p.add_argument('--physics-trace',action='store_true')
    p.add_argument('--snapshots',action='store_true')
    p.add_argument('--close-view',action='store_true')
    p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=352)
    run(p.parse_args())
