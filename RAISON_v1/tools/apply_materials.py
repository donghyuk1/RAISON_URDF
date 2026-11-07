"""Apply visual materials and separate palm triangles from the base visual.

Original STLs, collision geometry, joints and inertias are preserved.
"""
import csv
import io
import re
import struct
import xml.etree.ElementTree as ET
import numpy as np
from simplify_collisions import PACKAGE, URDF, read_stl
from contact_primitives import components

MATERIALS = {
    'printed_beige': '0.78 0.69 0.55 1',
    'aluminum': '0.72 0.75 0.79 1',
    'aluminum_blue_gray': '0.60 0.69 0.78 1',
    'metal_printed_gray': '0.40 0.42 0.45 1',
}


def material_for(name):
    if name in ['base_link', 'index_aa-mcp', 'ring_aa-mcp']:
        return 'aluminum_blue_gray'
    if name == 'palm_collision' or name.endswith('_distal'):
        return 'printed_beige'
    return 'metal_printed_gray' if name == 'thumb_metacarpal' else 'aluminum'


def main():
    data = read_stl(PACKAGE/'meshes/base_link.STL')
    candidates = [g for g in components(data['v'])
                  if np.ptp(g.reshape(-1, 3)[:, 2]) > .110]
    assert len(candidates) == 1 and len(candidates[0]) == 2610
    palm = candidates[0]
    reference = read_stl(PACKAGE/'meshes/FINGER_v6_structure_palm.STL')['v'].astype(float)*.001
    aligned = reference[:, :, [0, 2, 1]].copy(); aligned[:, :, 1] *= -1
    assert np.allclose(palm.min((0, 1)), aligned.min((0, 1)), atol=1e-7)
    assert np.allclose(palm.max((0, 1)), aligned.max((0, 1)), atol=1e-7)
    signatures = {t.tobytes() for t in palm}
    keep = np.array([t.tobytes() not in signatures for t in data['v']])
    assert np.count_nonzero(~keep) == len(palm)
    output = data[keep]
    (PACKAGE/'meshes/base_without_palm.STL').write_bytes(
        b'RAISON base visual without palm; source base_link.STL'.ljust(80, b' ')
        + struct.pack('<I', len(output)) + output.tobytes())

    text = URDF.read_text()
    root = ET.fromstring(text)
    assert not root.findall('material'), 'Materials already defined; edit the URDF palette directly.'
    declarations = '\n'.join(f'  <material name="{name}"><color rgba="{rgba}" /></material>'
                             for name, rgba in MATERIALS.items())
    text = re.sub(r'(<robot\b[^>]*>)', lambda m: m[0]+'\n'+declarations, text, count=1)
    for link in root.findall('link'):
        name = link.get('name')
        pattern = r'(<link\s+name="'+re.escape(name)+r'"[^>]*>)(.*?)(</link>)'
        def replace(match):
            body = match[2]
            if name == 'palm_collision':
                body = '''
    <visual>
      <origin xyz="0 0 0" rpy="1.5707963267949 0 0" />
      <geometry>
        <mesh filename="package://RAISON_v1/meshes/FINGER_v6_structure_palm.STL" scale="0.001 0.001 0.001" />
      </geometry>
      <material name="printed_beige" />
    </visual>''' + body
            else:
                body = re.sub(r'<material\b.*?</material>',
                              f'<material name="{material_for(name)}" />', body, flags=re.S)
                if name == 'base_link':
                    body = body.replace('meshes/base_link.STL', 'meshes/base_without_palm.STL')
            return match[1]+body+match[3]
        text, count = re.subn(pattern, replace, text, count=1, flags=re.S)
        assert count == 1
    text = text.replace('Collision-only fixed frame. Palm mass remains in base_link inertia.',
                        'Palm visual/collision frame. Palm mass remains in base_link inertia.')
    updated = ET.fromstring(text)
    for old in root.findall('link'):
        new = updated.find("link[@name='%s']" % old.get('name'))
        for tag in ['inertial', 'collision']:
            assert [ET.tostring(e) for e in old.findall(tag)] == [ET.tostring(e) for e in new.findall(tag)]
    assert [ET.tostring(j) for j in root.findall('joint')] == [ET.tostring(j) for j in updated.findall('joint')]
    URDF.write_text(text)
    path = PACKAGE/'urdf/RAISON_v1.csv'
    reader = csv.DictReader(path.open()); fields = reader.fieldnames; rows = list(reader)
    for row in rows:
        material = material_for(row['Link Name'])
        row['Material Name'] = material
        for channel, value in zip(['Red', 'Green', 'Blue', 'Alpha'], MATERIALS[material].split()):
            row['Color '+channel] = value
        if row['Link Name'] == 'base_link':
            row['Mesh Filename'] = 'package://RAISON_v1/meshes/base_without_palm.STL'
    buf = io.StringIO(); writer = csv.DictWriter(buf, fieldnames=fields, lineterminator='\n')
    writer.writeheader(); writer.writerows(rows); path.write_text(buf.getvalue())
    print('PASS: shared materials; palm separated; all joint, collision and inertial elements unchanged.')


if __name__ == '__main__':
    main()
