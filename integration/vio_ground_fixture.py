"""Optional visual-only textured ground fixture. No collision/flight commands.

Adds/removes ONLY the named integration fixture, preserving teammate models.
Surface is world z=0.001m; use matching flow ground datum in the runner.

Patterns (scene texture only, never a sensor measurement):
  checker  default, unchanged: alternating tiles of --tile-size-m.
           PERIODIC. Flow is ambiguous by one diagonal tile step once the
           ground moves more than about half a tile between two frames; run
           20261118-flightfbf7d8854ed1 (2 cm tiles) measured flow off by
           exactly such steps in flight. Use for static/ground checks only.
  noise    non-repeating multi-scale texture for flight: random detail from
           --tile-size-m up to 16x that, so it stays trackable from takeoff
           height to ~1.5 m and has no period to alias on.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess

NAME = 'integration_vio_ground_checker'   # unchanged: remove/cleanup use this name
MAX_PIXELS = 6400
CHECKER_PX_PER_TILE = 20
NOISE_PX_PER_FINEST = 8
NOISE_OCTAVES = 5                          # finest scale x1, x2, x4, x8, x16


def validate(size_m, tile_size_m):
    if not math.isfinite(tile_size_m) or not .01 <= tile_size_m <= 1:
        raise ValueError('Invalid tile scale')
    if not math.isfinite(size_m) or not 1 <= size_m <= 32:
        raise ValueError('Fixture size must be 1..32m')


def checker_texture(size_m, tile_size_m):
    import numpy as np
    validate(size_m, tile_size_m)
    pixels = int(round(size_m/tile_size_m))*CHECKER_PX_PER_TILE
    if not 20 <= pixels <= MAX_PIXELS:
        raise ValueError('Texture memory budget exceeded')
    yy, xx = np.indices((pixels, pixels))
    return np.where(((xx//CHECKER_PX_PER_TILE+yy//CHECKER_PX_PER_TILE) % 2) == 0, 210, 55).astype('uint8')


def noise_texture(size_m, tile_size_m, seed=0):
    """Sum of random value-noise octaves, contrast-stretched to 30..225.

    Each octave is an independent random grid with cell size tile*2^k,
    upsampled bicubically, so there is no repeat anywhere on the plane.
    """
    import cv2
    import numpy as np
    validate(size_m, tile_size_m)
    pixels = int(round(size_m/tile_size_m))*NOISE_PX_PER_FINEST
    if not NOISE_PX_PER_FINEST <= pixels <= MAX_PIXELS:
        raise ValueError('Texture memory budget exceeded')
    rng = np.random.default_rng(seed)
    image = np.zeros((pixels, pixels), np.float32)
    for k in range(NOISE_OCTAVES):
        cells = max(2, int(round(pixels/(NOISE_PX_PER_FINEST*2**k))))
        grid = rng.standard_normal((cells, cells)).astype(np.float32)
        image += cv2.resize(grid, (pixels, pixels), interpolation=cv2.INTER_CUBIC)*(.75**k)
    image -= image.mean()
    image *= 60/max(float(image.std()), 1e-6)
    return np.clip(image+127.5, 30, 225).astype('uint8')


def fixture_sdf(size_m, texture):
    return f'''<sdf version="1.9"><model name="{NAME}"><static>true</static><pose>0 0 0.001 0 0 0</pose><link name="visual_only"><visual name="checker"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>{size_m} {size_m}</size></plane></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>file://{texture.resolve()}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual></link></model></sdf>'''


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('action', choices=['create', 'remove'])
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--size-m', type=float, default=8.)
    p.add_argument('--tile-size-m', type=float, default=.1,
                   help='checker: tile size; noise: finest detail scale. Opt-in scene texture scale; never changes a sensor measurement')
    p.add_argument('--pattern', choices=['checker', 'noise'], default='checker')
    p.add_argument('--seed', type=int, default=0, help='noise pattern only; fixed for reproducible scenes')
    a = p.parse_args(argv)
    a.directory.mkdir(exist_ok=True, parents=True)
    if a.action == 'create':
        import cv2
        if a.pattern == 'checker':
            image, texture = checker_texture(a.size_m, a.tile_size_m), a.directory/'checker.png'
        else:
            image, texture = noise_texture(a.size_m, a.tile_size_m, a.seed), a.directory/'ground_noise.png'
        if not cv2.imwrite(str(texture), image):
            raise RuntimeError('Cannot write texture')
        sdf = fixture_sdf(a.size_m, texture)
        (a.directory/'fixture.sdf').write_text(sdf)
        payload = 'sdf: '+json.dumps(sdf)+' allow_renaming: false'
        service, request = '/world/iris_runway/create', 'gz.msgs.EntityFactory'
    else:
        payload = 'name: '+json.dumps(NAME)+' type: MODEL'
        service, request = '/world/iris_runway/remove', 'gz.msgs.Entity'
    result = subprocess.run(['gz', 'service', '-s', service, '--reqtype', request, '--reptype', 'gz.msgs.Boolean',
                             '--timeout', '5000', '--req', payload], capture_output=True, text=True, timeout=8)
    print(json.dumps({'action': a.action, 'name': NAME, 'request': payload, 'returncode': result.returncode,
                      'response': result.stdout, 'stderr': result.stderr}))
    if result.returncode or 'true' not in result.stdout.lower():
        raise SystemExit(1)


if __name__ == '__main__':
    main()
