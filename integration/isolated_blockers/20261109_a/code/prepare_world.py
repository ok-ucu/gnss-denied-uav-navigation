"""Integration-only resource overlay. Do not edit /opt or an existing world."""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

run = Path(sys.argv[1])
variant = sys.argv[2]
model_root = run / 'models' / variant
vehicle = model_root / 'iris_with_gimbal'
vehicle.mkdir(parents=True, exist_ok=False)
source = Path('/opt/ardupilot_gazebo/models/iris_with_gimbal/model.sdf')
text = source.read_text()
for axis in ('roll','pitch','yaw'):
    old = '<cmd_topic>/gimbal/cmd_' + axis + '</cmd_topic>'
    assert text.count(old) == 1
    text = text.replace(old, '<cmd_topic>/experiment/unused/' + axis + '</cmd_topic>')
(vehicle/'model.sdf').write_text(text)
(vehicle/'model.config').write_bytes(source.with_name('model.config').read_bytes())
world = Path('/opt/ardupilot_gazebo/worlds/iris_runway.sdf')
(run/'models'/f'{variant}_world.sdf').write_bytes(world.read_bytes())
gimbal = Path('/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf')
(run/'evidence'/f'{variant}_gimbal_original.sdf').write_bytes(gimbal.read_bytes())
meta = dict(variant=variant,original_vehicle_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            overlay_sha256=hashlib.sha256(text.encode()).hexdigest(),
            changes=['Redirect only plugin control cmd_topic for '+a for a in ('roll','pitch','yaw')],
            joint_controllers_preserved=True,translations_preserved=True,
            execution_allowed=False,forwarding_enabled=False)
(run/'evidence'/f'{variant}_overlay.json').write_text(json.dumps(meta,indent=2)+'\n')
ET.fromstring(text)
