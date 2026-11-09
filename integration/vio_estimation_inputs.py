"""Explicit vertical aid + fixed extrinsics; never read GT XY or attitude."""
import json
import math
from pathlib import Path


def fixed_mount(path):
    data=json.loads(Path(path).read_text())
    angles=data['max_angles_deg']
    lever=data['final_lever_body_flu_m']
    if (data.get('mount_status')!='PASS' or len(angles)!=3 or len(lever)!=3
            or not all(math.isfinite(x) for x in angles+lever)
            or any(not 0<=x<=1 for x in angles)):
        raise ValueError('Require measured <=1degree calibration')
    return [lever[0],-lever[1],-lever[2]]


def vertical_inputs(root_height_enu,ground_z,body_to_ned,lever_frd):
    import numpy as np
    values=np.asarray([root_height_enu,ground_z,*lever_frd],float)
    r=np.asarray(body_to_ned,float)
    if not np.isfinite(values).all() or r.shape!=(3,3) or not np.isfinite(r).all():
        raise ValueError('Nonfinite vertical aid or IMU rotation')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('Invalid IMU rotation')
    height=float(root_height_enu-ground_z)
    camera_height=height-float((r@np.asarray(lever_frd))[2])
    return height,camera_height
