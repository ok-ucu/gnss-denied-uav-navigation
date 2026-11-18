"""Actual pose-hierarchy measurement; no renderer, publishers or control API."""
import math

def mm(a,b):return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def transpose(a):return [list(c) for c in zip(*a)]
def mv(a,b):return [sum(x*y for x,y in zip(row,b)) for row in a]

def quaternion_matrix(q):
    if len(q)!=4 or not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1)>1e-5:
        raise ValueError('Invalid quaternion')
    x,y,z,w=q
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]

def measure_mount(body,gimbal,pitch,sensor_rotation,sensor_translation):
    rb,rg,rp=[quaternion_matrix(p['quaternion_xyzw']) for p in (body,gimbal,pitch)]
    for p in (body,gimbal,pitch):
        if len(p['position'])!=3 or not all(math.isfinite(v) for v in p['position']):
            raise ValueError('Invalid position')
    rc=mm(mm(mm(transpose(rb),rg),rp),sensor_rotation)
    axes=[[rc[i][0] for i in range(3)],[-rc[i][1] for i in range(3)],[-rc[i][2] for i in range(3)]]
    expected=[[0,0,-1],[0,-1,0],[-1,0,0]]
    angles=[math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b)))))) for a,b in zip(axes,expected)]
    pitch_offset=[x+y for x,y in zip(pitch['position'],mv(rp,sensor_translation))]
    pcam=[x+y-z for x,y,z in zip(gimbal['position'],mv(rg,pitch_offset),body['position'])]
    return dict(angles_deg=angles,lever_body_flu_m=mv(transpose(rb),pcam),rotation_body_flu_sensor=rc)

def mount_pass(samples,tolerance_deg=1):
    if not math.isfinite(tolerance_deg) or not 0<tolerance_deg<=1:raise ValueError('Invalid tolerance')
    return bool(samples) and all(len(s['angles_deg'])==3 and all(math.isfinite(x) and 0<=x<=tolerance_deg for x in s['angles_deg']) for s in samples)
