"""Locate the team's VIO estimator sources without copying them into this branch.

`vio/` lives on the `vio-setup` branch and is not merged into the integration
branches. The offline tools import the UNMODIFIED estimator files from:

  1. --vio-dir / $VIO_SOURCE_DIR, when given;
  2. <repo>/vio, when that directory exists in the checkout;
  3. otherwise the pinned commit below, exported with `git show` into
     vio_offline/_vio_src/<commit>/ (git-ignored).

The pin and file hashes are the same ones integration/linux_gpu/prepare_vio.py uses.
"""
import hashlib
import os
import subprocess
import sys
from pathlib import Path

PIN = "d33f66008fd7b6de332c96dd324b8ddbd320c194"      # origin/vio-setup, 2026-11-07
FILES = {
    "vio_core.py": "1bddfbbcf12fad27bf5c769a1718f11d9592f2bee54bdb238781b11f5dc40efa",
    "vio_frontend.py": "db69d2c5a11ce2ad20cc3f94d78f35aa2b07b6755931372b7e5f3fc0a3d11682",
}
REPO = Path(__file__).resolve().parents[1]


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _export_pinned():
    out = Path(__file__).resolve().parent / "_vio_src" / PIN
    if not all((out / name).exists() for name in FILES):
        out.mkdir(parents=True, exist_ok=True)
        for name in FILES:
            data = subprocess.check_output(["git", "show", f"{PIN}:vio/{name}"], cwd=REPO)
            (out / name).write_bytes(data)
    return out


def locate(vio_dir=None):
    """Return (directory, info dict). Adds the directory to sys.path."""
    vio_dir = vio_dir or os.environ.get("VIO_SOURCE_DIR")
    if vio_dir:
        directory, origin = Path(vio_dir).resolve(), "explicit"
    elif (REPO / "vio" / "vio_core.py").exists():
        directory, origin = REPO / "vio", "checkout"
    else:
        directory, origin = _export_pinned(), f"git:{PIN}"
    hashes = {name: _sha(directory / name) for name in FILES}
    info = dict(directory=str(directory), origin=origin, pinned_commit=PIN, sha256=hashes,
                matches_pin=all(hashes[n] == FILES[n] for n in FILES))
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
    return directory, info
