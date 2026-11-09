#!/usr/bin/env python3
"""Захоплення кадрів з міткою часу годинника бортового комп'ютера.

Мітка береться ОДРАЗУ після grab(), до декодування (retrieve), щоб
затримка декодування не потрапила в час кадру. Годинник - CLOCK_MONOTONIC
(time.monotonic_ns), той самий, що в dataset/recorder.py.

Окремо: python3 capture.py --config camera_config.yaml --preview
"""
import argparse
import os
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))

import cv2
import numpy as np
import yaml


def load_config(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def apply_v4l2_ctrls(cfg):
    if cfg.get("backend") != "v4l2":
        return
    dev = str(cfg.get("device", "/dev/video0"))
    if dev.isdigit():
        dev = f"/dev/video{dev}"
    for c in cfg.get("v4l2_ctrls") or []:
        try:
            subprocess.run(["v4l2-ctl", "-d", dev, "-c", c], check=True,
                           capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError) as e:
            print(f"УВАГА: v4l2-ctl -c {c} не спрацювало: {e}")


class Camera:
    """read() -> (t_ns, gray uint8) або (None, None), якщо кадру нема."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.backend = cfg.get("backend", "v4l2")
        self.w, self.h = int(cfg["width"]), int(cfg["height"])
        self.fps = float(cfg.get("fps", 30))
        self.cap = None
        self._n = 0
        self._t_next = None
        if self.backend == "synthetic":
            rng = np.random.default_rng(0)
            tex = rng.integers(0, 255, (self.h + 400, self.w + 400), dtype=np.uint8)
            self._tex = cv2.GaussianBlur(tex, (0, 0), 2.0)
        elif self.backend == "gstreamer":
            self.cap = cv2.VideoCapture(cfg["gstreamer_pipeline"], cv2.CAP_GSTREAMER)
        elif self.backend in ("v4l2", "video"):
            apply_v4l2_ctrls(cfg)
            dev = cfg.get("device", 0)
            if self.backend == "v4l2":
                dev = int(dev) if str(dev).isdigit() else dev
                self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                fourcc = cfg.get("fourcc")
                if fourcc:
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            else:
                self.cap = cv2.VideoCapture(str(dev))
        else:
            raise ValueError(f"невідомий backend: {self.backend}")
        if self.cap is not None and not self.cap.isOpened():
            raise RuntimeError("камера не відкрилась (device/pipeline/права?)")

    def read(self):
        if self.backend == "synthetic":
            period = 1.0 / self.fps
            now = time.monotonic()
            if self._t_next is None:
                self._t_next = now
            if self._t_next > now:
                time.sleep(self._t_next - now)
            self._t_next += period
            t_ns = time.monotonic_ns()
            x = 100 + int(self._n * 2) % 300
            y = 100 + int(self._n * 1) % 300
            self._n += 1
            return t_ns, self._tex[y:y + self.h, x:x + self.w].copy()
        if not self.cap.grab():
            return None, None
        t_ns = time.monotonic_ns()           # мітка до декодування
        ok, frame = self.cap.retrieve()
        if not ok:
            return None, None
        if frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame.shape != (self.h, self.w):
            # камера віддала інший розмір, ніж у конфігу => калібрування не підійде
            if not getattr(self, "_warned", False):
                print(f"УВАГА: розмір кадру {frame.shape[::-1]} != "
                      f"{(self.w, self.h)} з конфігу")
                self._warned = True
        return t_ns, frame

    def release(self):
        if self.cap is not None:
            self.cap.release()


class CaptureThread(threading.Thread):
    """Читає кадри у фоні й віддає у callback(t_ns, gray)."""

    def __init__(self, cam, callback):
        super().__init__(daemon=True)
        self.cam, self.callback = cam, callback
        self.stop_flag = threading.Event()
        self.n_fail = 0

    def run(self):
        while not self.stop_flag.is_set():
            t_ns, img = self.cam.read()
            if img is None:
                self.n_fail += 1
                time.sleep(0.005)
                continue
            self.callback(t_ns, img)

    def stop(self):
        self.stop_flag.set()
        self.join(timeout=2.0)
        self.cam.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "camera_config.yaml"))
    ap.add_argument("--synthetic", action="store_true", help="використовувати синтетичну камеру (тест без заліза)")
    ap.add_argument("--preview", action="store_true", help="показати вікно (потрібен дисплей)")
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()
    cfg = load_config(args.config) if os.path.exists(args.config) else {
        "backend": "synthetic", "width": 640, "height": 480, "fps": 30
    }
    if args.synthetic:
        cfg["backend"] = "synthetic"
    cam = Camera(cfg)
    ts, t_end = [], time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        t_ns, img = cam.read()
        if img is None:
            continue
        ts.append(t_ns)
        if args.preview:
            cv2.imshow("camera", img)
            if cv2.waitKey(1) == 27:
                break
    cam.release()
    if len(ts) > 2:
        d = np.diff(ts) * 1e-6
        print(f"кадрів {len(ts)}, fps {1000.0 / d.mean():.1f}, "
              f"інтервал мс: середній {d.mean():.1f}, макс {d.max():.1f}, "
              f"розкид (std) {d.std():.2f}")
    else:
        print("кадрів майже немає - перевір камеру")


if __name__ == "__main__":
    main()
