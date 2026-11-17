"""Optional visual-only textured ground fixture. No collision/flight commands.

Adds/removes ONLY the named integration fixture, preserving teammate models.
Surface is world z=0.001m; use matching flow ground datum in the runner.
"""
import argparse
import json
from pathlib import Path
import subprocess

p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['create','remove']);p.add_argument('--directory',type=Path,required=True);p.add_argument('--size-m',type=float,default=8.)
p.add_argument('--tile-size-m',type=float,default=.1,help='Opt-in scene texture scale; never changes a sensor measurement')
a=p.parse_args()
a.directory.mkdir(exist_ok=True,parents=True)
name='integration_vio_ground_checker'
if a.action=='create':
    import cv2
    import numpy as np
    # 8m square, alternating 0.1m tiles; static scene texture, not sensor data.
    import math
    if not math.isfinite(a.tile_size_m) or not .01<=a.tile_size_m<=1:raise ValueError('Invalid tile scale')
    pixels=int(round(a.size_m/a.tile_size_m))*20
    if not 20<=pixels<=6400:raise ValueError('Texture memory budget exceeded')
    if not 1<=a.size_m<=32:raise ValueError('Fixture size must be 1..32m')
    yy,xx=np.indices((pixels,pixels));image=np.where(((xx//20+yy//20)%2)==0,210,55).astype('uint8')
    texture=a.directory/'checker.png'
    if not cv2.imwrite(str(texture),image):raise RuntimeError('Cannot write texture')
    sdf=f'''<sdf version="1.9"><model name="{name}"><static>true</static><pose>0 0 0.001 0 0 0</pose><link name="visual_only"><visual name="checker"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>{a.size_m} {a.size_m}</size></plane></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>file://{texture.resolve()}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual></link></model></sdf>'''
    (a.directory/'fixture.sdf').write_text(sdf)
    payload='sdf: '+json.dumps(sdf)+' allow_renaming: false'
    service='/world/iris_runway/create';request='gz.msgs.EntityFactory'
else:
    payload='name: '+json.dumps(name)+' type: MODEL';service='/world/iris_runway/remove';request='gz.msgs.Entity'
result=subprocess.run(['gz','service','-s',service,'--reqtype',request,'--reptype','gz.msgs.Boolean','--timeout','5000','--req',payload],capture_output=True,text=True,timeout=8)
print(json.dumps({'action':a.action,'name':name,'request':payload,'returncode':result.returncode,'response':result.stdout,'stderr':result.stderr}))
if result.returncode or 'true' not in result.stdout.lower():raise SystemExit(1)
