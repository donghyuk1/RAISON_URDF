"""Rebuild collision proxies from visual STLs; requires Python and NumPy.

Run from any directory. Rebuilding overwrites manual collision edits, but
preserves joints, visual geometry and inertia. Dimensions are in meters.
"""
from pathlib import Path
import math
from fit_collision_primitives import rpy
from contact_primitives import build
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / 'urdf/RAISON_v1.urdf'
DTYPE = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
BASE_RADIUS = 0.021
BASE_LENGTH = 0.030
BASE_CENTER = [0.0185, -0.0803, -0.005]


def read_stl(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    assert len(raw) == 84 + 50 * count, path
    return np.frombuffer(raw, dtype=DTYPE, offset=84, count=count)


def fmt(values):
    return ' '.join(f'{float(x):.10g}' for x in values)


def collision(name, xyz, rpy, shape, **dimensions):
    c = ET.Element('collision', name=name)
    ET.SubElement(c, 'origin', xyz=fmt(xyz), rpy=fmt(rpy))
    g = ET.SubElement(c, 'geometry')
    ET.SubElement(g, shape, **dimensions)
    return c


def finger_collisions(name, triangles):
    result = []
    for i, proxy in enumerate(build(name, triangles)):
        dimensions = ({'size': fmt(proxy['size'])} if proxy['shape'] == 'box'
                      else {'radius': fmt([proxy['radius']]), 'length': fmt([proxy['length']])})
        result.append(collision(proxy['name'], proxy['xyz'],
                                rpy(proxy['rotation']), proxy['shape'], **dimensions))
    print(name, len(result), 'primitives', flush=True)
    return result


def main():
    text = URDF.read_text()
    root = ET.fromstring(text)
    cache = {}
    for link in root.findall('link'):
        name = link.get('name')
        if name == 'palm_collision':
            proxies = [collision('palm_mesh', [0, 0, 0], [math.pi / 2, 0, 0],
                                 'mesh', filename='package://RAISON_v1/meshes/FINGER_v6_structure_palm.STL',
                                 scale='0.001 0.001 0.001')]
        elif name == 'base_link':
            proxies = [collision('base_actuator_cylinder',
                                 BASE_CENTER,
                                 [math.pi / 2, 0, 0],
                                 'cylinder', radius=str(BASE_RADIUS), length=str(BASE_LENGTH))]
        else:
            mesh = link.find('visual/geometry/mesh').get('filename')
            data = read_stl(PACKAGE / mesh.split('package://RAISON_v1/')[1])
            visual_origin = link.find('visual/origin')
            assert np.allclose(list(map(float, visual_origin.get('xyz').split())), 0)
            assert np.allclose(list(map(float, visual_origin.get('rpy').split())), 0)
            if mesh not in cache:
                cache[mesh] = finger_collisions(name, data['v'])
            proxies = cache[mesh]
        fragments = []
        for proxy in proxies:
            ET.indent(proxy, space='  ', level=2)
            fragments.append('    ' + ET.tostring(proxy, encoding='unicode').strip())
        pattern = r'(<link\s+name="' + re.escape(name) + r'"[^>]*>)(.*?)(</link>)'
        def replace(match):
            body = re.sub(r'\s*<collision\b.*?</collision>', '', match[2], flags=re.S)
            return match[1] + body.rstrip() + '\n' + '\n'.join(fragments) + '\n  ' + match[3]
        text, changed = re.subn(pattern, replace, text, count=1, flags=re.S)
        assert changed == 1
    URDF.write_text(text, encoding='utf-8')
    print('Updated compound primitives and aligned palm mesh.')


if __name__ == '__main__':
    main()
