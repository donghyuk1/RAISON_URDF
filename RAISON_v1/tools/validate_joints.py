"""Verify requested joint limits, effort, speed, CSV consistency and motion signs."""
import csv
import math
import xml.etree.ElementTree as ET
import numpy as np
from simplify_collisions import PACKAGE, URDF
from validate_collisions import rotation

root = ET.parse(URDF).getroot()
joints = {j.get('name'): j for j in root.findall('joint')}
rows = {r['Joint Name']: r for r in csv.DictReader((PACKAGE/'urdf/RAISON_v1.csv').open())}
frames = {'base_link': np.eye(4)}
joint_frames = {}
pending = list(joints.values())
while pending:
    progress = False
    for j in pending[:]:
        parent = j.find('parent').get('link')
        if parent not in frames:
            continue
        transform = np.eye(4)
        transform[:3, :3] = rotation(j.find('origin'))
        transform[:3, 3] = np.fromstring(j.find('origin').get('xyz'), sep=' ')
        world = frames[parent] @ transform
        frames[j.find('child').get('link')] = world
        joint_frames[j.get('name')] = world
        pending.remove(j); progress = True
    assert progress, 'Disconnected or cyclic joint tree'

movable = 0
for name, j in joints.items():
    if j.get('type') == 'fixed':
        assert j.find('limit') is None
        continue
    movable += 1
    assert j.get('type') == 'revolute'
    kind = 'abad' if 'abad' in name else name.rsplit('_', 1)[1]
    ranges = ({'CMC': (-10, 90), 'MCP': (-20, 60), 'IP': (-20, 90)}
              if 'thumb' in name else
              {'abad': (-20, 20), 'MCP': (-10, 90), 'PIP': (0, 110), 'DIP': (0, 80)})
    if name == 'joint_ring_abad':
        ranges['abad'] = (-20, 10)
    limit = j.find('limit').attrib
    assert np.allclose([float(limit['lower']), float(limit['upper'])],
                       np.radians(ranges[kind]), atol=1e-11, rtol=0)
    assert float(limit['effort']) == {'PIP': 1, 'DIP': .5}.get(kind, 2)
    assert float(limit['velocity']) == 10
    axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
    assert np.isclose(np.linalg.norm(axis), 1)
    row = rows[name]
    assert row['Joint Type'] == 'revolute'
    assert np.allclose(axis, [float(row['Joint Axis '+a]) for a in 'XYZ'])
    for key, value in limit.items():
        assert float(row['Limit '+key.capitalize()]) == float(value)
    if 'thumb' in name:
        expected = [0, 0, 1] if kind == 'CMC' else [math.cos(math.pi/9), math.sin(math.pi/9), 0]
        assert np.allclose(axis, expected, atol=1e-12)
        continue
    finger = next(f for f in ['index', 'middle', 'ring'] if f in name)
    tip = frames[finger+'_distal'] @ np.array([0, 0, -.02, 1])
    world = joint_frames[name]
    velocity = np.cross(world[:3, :3] @ axis, tip[:3]-world[:3, 3])
    if kind == 'abad':
        # Base -Z is index-side outward; +Z is ring-side outward.
        assert velocity[2] * (-1 if finger == 'index' else 1) > 0
    else:
        # Positive flexion moves the extended fingertip toward the palm's +X side.
        assert velocity[0] > 0
assert movable == 14
assert not any('middle' in n and 'abad' in n for n in joints)
print('PASS: 14 bounded joints, requested angle/effort limits, 10 rad/s velocity, CSV consistency, positive outward/flexion signs, unchanged thumb directions.')
