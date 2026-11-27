"""Preflight a block-to-cup task; no serial or Tk calls in this module."""
from dataclasses import dataclass, asdict
import math
import numpy as np
import miniarm_ik as ik


@dataclass(frozen=True)
class CupSettings:
    x: float = 127.0
    y: float = 0.0
    rim_z: float = 127.0
    diameter: float = 101.6
    lift: float = 40.0
    object_radius: float = 30.0
    release_margin: float = 15.0
    dwell: float = 0.8

    def validate(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Cup settings must be finite numbers')
        if not 0 < self.rim_z <= 300 or not 10 <= self.diameter <= 300:
            raise ValueError('Enter a positive measured cup rim height and opening diameter (mm)')
        if not 20 <= self.lift <= 150 or not 5 <= self.object_radius <= 100:
            raise ValueError('Lift must be 20..150 mm; held-block radius must be 5..100 mm')
        if not 10 <= self.release_margin <= 100 or not .2 <= self.dwell <= 5:
            raise ValueError('Release margin must be 10..100 mm; settling pause .2..5 seconds')
        # Leave space for the 10 mm rounded-command miss limit plus 5 mm margin.
        if self.object_radius + 15 >= self.diameter / 2:
            raise ValueError('Block radius plus positioning margin does not fit the cup opening')
        return self

    @property
    def release_xyz(self):
        return np.array([self.x, self.y, self.rim_z+self.object_radius+self.release_margin])


@dataclass
class Step:
    label: str
    pose: list
    duration_ms: int
    pause_ms: int
    xyz: np.ndarray
    holding: bool
    result: dict | None = None


@dataclass
class Plan:
    start: list
    pick: np.ndarray
    settings: CupSettings
    seconds: float
    steps: list


def box_distance(point, low, high):
    return float(np.linalg.norm(np.maximum(np.maximum(low-point, point-high), 0.)))


def segment_box_distance(a, b, low, high):
    """Exact segment/AABB distance by piecewise quadratic minimization."""
    a=np.asarray(a); b=np.asarray(b); delta=b-a
    knots={0.,1.}
    for i in range(3):
        if abs(delta[i])>1e-15:
            for face in (low[i],high[i]):
                t=(face-a[i])/delta[i]
                if 0<t<1: knots.add(float(t))
    knots=sorted(knots); best=min(box_distance(a,low,high),box_distance(b,low,high))
    for left,right in zip(knots[:-1],knots[1:]):
        middle=a+(left+right)*.5*delta
        mask=(middle<low)|(middle>high)
        bound=np.where(middle<low,low,high)
        v=delta[mask]; offset=(a-bound)[mask]
        t=left if v@v<1e-24 else float(np.clip(-(offset@v)/(v@v),left,right))
        best=min(best,box_distance(a+t*delta,low,high))
    return best


def cup_clearance(m,q,settings,holding=False):
    """Conservative solid cup envelope; never insert the claw into the cup.

    An AABB encloses the cup, with 5 mm lip/body allowance beyond the opening.
    Robot links use existing capsule radii; a held block uses a TCP sphere.
    This remains an approximation, not a model of fingers/contact/physics.
    """
    P=ik.fk(m,q)[1]*1000
    radius=settings.diameter/2+5
    low=np.array([settings.x-radius,settings.y-radius,0.])
    high=np.array([settings.x+radius,settings.y+radius,settings.rim_z])
    if m.collision is None:
        raise ValueError('Pick-and-place requires the configured robot collision model')
    c=m.collision; margin=max(3.,1000*c['margin'])
    values=[segment_box_distance(P[a],P[b],low,high)-1000*r-margin
            for (a,b),r in zip(c['edges'],c['radii'])]
    if holding:
        values.append(box_distance(P[-1],low,high)-settings.object_radius-margin)
    return min(values)


def validate_cup_leg(m,cal,start,end,settings,holding=False,cancel=lambda:False):
    """Certify continuous raw interpolation, including calibration knots."""
    start=np.asarray(start,dtype=float); end=np.asarray(end,dtype=float)
    delta=end-start; fractions={0.,1.}
    for name in ik.JOINTS:
        index=ik.mc.SERVO_INDEX[name]
        if delta[index]:
            for raw,_ in cal.points[name]:
                f=(raw-start[index])/delta[index]
                if 0<f<1: fractions.add(float(f))
    reach=1000*(np.sum(np.abs(m.mdh[:,0]))+np.sum(np.abs(m.mdh[:,2]))+
                np.linalg.norm(m.tool[:3,3]))
    def check(a,b,depth=0):
        if cancel(): raise ValueError('Pick-and-place planning cancelled')
        ca=cup_clearance(m,a,settings,holding); cb=cup_clearance(m,b,settings,holding)
        if min(ca,cb)<=0: return False
        # Distance to a fixed obstacle is 1-Lipschitz in point travel.
        if max(ca,cb)>reach*np.sum(np.abs(b-a))+1e-9: return True
        if depth>=12: return False
        mid=(a+b)*.5
        return check(a,mid,depth+1) and check(mid,b,depth+1)
    previous=ik.servo_to_q(cal,start)
    for fraction in sorted(fractions)[1:]:
        current=ik.servo_to_q(cal,start+fraction*delta)
        if not check(previous,current):
            raise ValueError('Path intersects the cup/held-block clearance envelope')
        previous=current


def seeds(m,target,current):
    aim=math.degrees(math.atan2(target[1],target[0]))
    guesses=[current, np.deg2rad([aim,60,-60,-60]),
             np.deg2rad([aim,90,-45,-45]), np.deg2rad([aim,30,-60,-30]), m.q0]
    result=[]
    for q in guesses:
        q=np.clip(q,m.limits[:,0],m.limits[:,1])
        if not any(np.allclose(q,p) for p in result): result.append(q)
    return result


def build_plan(m,cal,start,pick,settings,seconds=2.,cancel=lambda:False,progress=lambda s:None):
    """Solve and check EVERY stage before returning anything executable."""
    settings.validate(); pick=np.asarray(pick,dtype=float)
    if pick.shape!=(3,) or not np.isfinite(pick).all() or pick[2]<=0:
        raise ValueError('Pickup XYZ must contain three finite values, with Z above the table')
    ik.motion_duration_ms(start,start,seconds)
    plan=Plan(list(start),pick.copy(),settings,float(seconds),[])
    pose=list(start)
    def add(label,final,holding,result=None,pause=0):
        nonlocal pose
        if cancel(): raise ValueError('Pick-and-place planning cancelled')
        ik.validate_coordinated_leg(m,cal,pose,final)
        validate_cup_leg(m,cal,pose,final,settings,holding,cancel)
        xyz=ik.position(m,ik.servo_to_q(cal,final))*1000
        plan.steps.append(Step(label,list(final),ik.motion_duration_ms(pose,final,seconds),
                               int(pause),xyz.copy(),holding,result))
        pose=list(final)
    def grip(label,closed):
        progress(label)
        final=list(pose); final[0]=cal.claw_closed if closed else cal.claw_open
        add(label,final,closed,pause=round(settings.dwell*1000))
    def move(label,target,holding):
        nonlocal pose
        # Prefer one smooth command for the entire stage. If no checked direct
        # route is found, retry with shorter Cartesian waypoints.
        origin=ik.position(m,ik.servo_to_q(cal,pose))*1000
        initial=list(pose); first=len(plan.steps)
        counts=[1,max(2,int(math.ceil(np.linalg.norm(target-origin)/20)))]
        for count in counts:
            pose=list(initial); del plan.steps[first:]
            for k in range(1,count+1):
                xyz=origin+(np.asarray(target)-origin)*(k/count)
                progress(f'{label} ({k}/{count})')
                q0=ik.servo_to_q(cal,pose); failure='No IK solution found'
                for seed in seeds(m,xyz,q0):
                    if cancel(): raise ValueError('Pick-and-place planning cancelled')
                    try:
                        q,r=ik.inverse_kinematics(xyz,m,seed)
                        if not r['success']:
                            failure=r['status']; continue
                        checked=dict(r,history=np.vstack([q0,q]))
                        final,miss=ik.plan_motion(m,cal,pose,xyz,checked,10)
                        add(f'{label} ({k}/{count})',final[0],holding,checked,
                            pause=round(settings.dwell*1000) if k==count else 0)
                        break
                    except ValueError as error: failure=str(error)
                else:
                    break
            else:
                return
        raise ValueError(f'{label}: {failure}. Reposition the block/cup or adjust clearance; no motion was sent.')
    release=settings.release_xyz
    # Lift at the pickup XY BEFORE crossing toward the cup. A diagonal
    # transfer from the low pickup approach to the cup can intersect the
    # cup envelope despite the endpoint being above the rim.
    #
    # The lift setting is the initial block pickup lift; it is NOT added
    # again to the high cup release height, since that can exceed reach.
    # A small (5 mm) extra Z clearance above release makes the approach
    # and retreat separate from the release setpoint. Continuous servo
    # motion, joint limits, floor and cup collisions are still verified.
    pickup_approach_z = float(pick[2] + settings.lift)
    transfer_z = float(max(pickup_approach_z, release[2] + 5.0))
    above_pick = np.array([pick[0],pick[1],pickup_approach_z])
    high_above_pick = np.array([pick[0],pick[1],transfer_z])
    above_cup = np.array([settings.x,settings.y,transfer_z])

    grip('Open claw',False)
    move('Approach above block',above_pick,False)
    move('Lower to pickup',pick,False)
    grip('Close claw / grasp',True)
    move('Lift block',above_pick,True)
    # Critical order: finish the vertical lift at pickup XY first.
    if transfer_z > pickup_approach_z + 1e-9:
        move('Raise block clear of cup',high_above_pick,True)
    move('Cross horizontally above cup',above_cup,True)
    move('Lower to release above rim',release,True)
    grip('Open claw / release',False)
    move('Retreat above cup',above_cup,False)
    return plan
