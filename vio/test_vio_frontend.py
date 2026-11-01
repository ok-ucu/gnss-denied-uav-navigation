import numpy as np
import cv2

from vio_core import rot_body_to_ned
from vio_frontend import FlowFrontend

F, H, DT = 500.0, 10.0, 0.05


def texture(seed=0, size=1600):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, (size, size), dtype=np.uint8)
    return cv2.GaussianBlur(img, (0, 0), 2.0)


def frames(dx, dy, n=6, size=1600):
    tex = texture(size=size)
    for i in range(n):
        x0, y0 = 400 + int(i * dx), 400 + int(i * dy)
        yield tex[y0:y0 + 480, x0:x0 + 640].copy()


def run(dx, dy, gyro=(0, 0, 0)):
    fe = FlowFrontend(fx=F)
    R = rot_body_to_ned(0, 0, 0)
    res = None
    for i, fr in enumerate(frames(dx, dy)):
        res = fe.process(fr, i * DT, H, np.array(gyro), R)
    return res


def test_move_right_gives_east_velocity():
    """вікно їде вправо по текстурі на 3 px/кадр => дрон летить вправо
    v = (h/f) * 3/dt = 0.02 * 60 = 1.2 м/с (курс 0 => вправо = схід)"""
    res = run(3, 0)
    assert res is not None and res.n_inliers > 30
    assert abs(res.v_xy[1] - 1.2) < 0.1 and abs(res.v_xy[0]) < 0.1


def test_move_forward_gives_north_velocity():
"""    вісь y зображення = назад. Земля тече назад (потік y+) => дрон летить вперед.
    Вікно їде ВГОРУ по текстурі (dy<0) => об'єкти зміщуються вниз в кадрі"""
    res = run(0, -3)
    assert abs(res.v_xy[0] - 1.2) < 0.1 and abs(res.v_xy[1]) < 0.1


def test_flat_image_is_rejected():
    fe = FlowFrontend(fx=F)
    R = rot_body_to_ned(0, 0, 0)
    flat = np.full((480, 640), 120, np.uint8)
    out = [fe.process(flat, i * DT, H, np.zeros(3), R) for i in range(3)]
    assert all(o is None for o in out)
