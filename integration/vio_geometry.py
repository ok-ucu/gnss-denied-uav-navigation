"""Rigid sensor mounting corrections; no GT horizontal state input."""


def camera_to_body_xy(camera_velocity_xy, body_to_ned, gyro_frd, camera_lever_frd):
    import numpy as np
    velocity=np.asarray(camera_velocity_xy,dtype=float)
    r=np.asarray(body_to_ned,dtype=float)
    omega=np.asarray(gyro_frd,dtype=float)
    lever=np.asarray(camera_lever_frd,dtype=float)
    if velocity.shape!=(2,) or r.shape!=(3,3) or omega.shape!=(3,) or lever.shape!=(3,):
        raise ValueError('Invalid velocity/rotation/gyro/lever shape')
    if not all(np.all(np.isfinite(x)) for x in (velocity,r,omega,lever)):
        raise ValueError('Nonfinite rigid-body transform input')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(r),1,atol=1e-5):
        raise ValueError('Expected proper body-to-NED rotation')
    return velocity-(r@np.cross(omega,lever))[:2]
