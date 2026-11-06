"""Check collision coverage/alignment and render a review overlay."""
import itertools
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw
from simplify_collisions import PACKAGE, URDF, read_stl
from fit_collision_primitives import subdivide


def rotation(origin):
    r, p, y = map(float, origin.get('rpy', '0 0 0').split())
    cr, cp, cy = np.cos([r, p, y]); sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def pose(c):
    o = c.find('origin')
    return np.fromstring(o.get('xyz'), sep=' '), rotation(o)


def volume(c):
    b, cy = c.find('geometry/box'), c.find('geometry/cylinder')
    if b is not None:
        return float(np.prod(np.fromstring(b.get('size'), sep=' ')))
    if cy is not None:
        return math.pi * float(cy.get('radius'))**2 * float(cy.get('length'))
    return None


def main():
    root = ET.parse(URDF).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    report = {}
    old = json.loads((PACKAGE/'validation/bounding_fit_volumes.json').read_text())
    snapshot = json.loads((PACKAGE/'validation/pre_refinement_properties.json').read_text())
    assert [ET.tostring(j, encoding='unicode') for j in root.findall('joint')] == snapshot['joints']
    parents = {j.find('child').get('link'): j.find('parent').get('link') for j in root.findall('joint')}
    assert len(parents) == len(links)-1
    for name in links:
        seen = set(); current = name
        while current in parents:
            assert current not in seen
            seen.add(current); current = parents[current]
        assert current == 'base_link'
    for mesh in root.findall('.//mesh'):
        assert (PACKAGE/mesh.get('filename').split('package://RAISON_v1/')[1]).is_file()
    assert len(root.findall('.//collision/geometry/mesh')) == 1
    for name, link in links.items():
        for tag in ['visual', 'inertial']:
            element = link.find(tag)
            assert (ET.tostring(element, encoding='unicode') if element is not None else None) == snapshot[name][tag]
        if name in ['base_link', 'palm_collision']:
            continue
        mesh = link.find('visual/geometry/mesh').get('filename').split('package://RAISON_v1/')[1]
        triangles = subdivide(read_stl(PACKAGE/mesh)['v'])
        covered = np.zeros(len(triangles), dtype=bool)
        collisions = link.findall('collision')
        for c in collisions:
            center, frame = pose(c)
            local = (triangles - center) @ frame
            b, cy = c.find('geometry/box'), c.find('geometry/cylinder')
            if b is not None:
                size = np.fromstring(b.get('size'), sep=' ')
                assert np.all(size > 0)
                inside = np.all(np.abs(local) <= size/2+1e-8, axis=2)
            else:
                radius, length = float(cy.get('radius')), float(cy.get('length'))
                assert radius > 0 and length > 0
                inside = ((np.linalg.norm(local[:, :, :2], axis=2) <= radius+1e-8)
                          & (np.abs(local[:, :, 2]) <= length/2+1e-8))
            covered |= inside.all(1)
        total = sum(volume(c) for c in collisions)
        report[name] = dict(primitives=len(collisions), subdivided_faces=len(triangles),
                            enclosed_subtriangle_fraction=float(covered.mean()), summed_volume_m3=total,
                            summed_volume_reduction_percent=100*(1-total/old[name]))
    # Contact faces must coincide with their source planes, not enclosing planes.
    from contact_primitives import thumb_contact_plane
    source = read_stl(PACKAGE/'meshes/thumb_distal.STL')['v'].astype(float)
    tip = links['thumb_distal']
    for target, name in [([-.0885, .2432, -.9659], 'thumb_front_contact'),
                         ([.1932, -.5307, .8252], 'thumb_back_contact')]:
        expected = thumb_contact_plane(source, target, name)
        c = tip.find("collision[@name='%s']" % name)
        center, frame = pose(c)
        half = np.fromstring(c.find('geometry/box').get('size'), sep=' ')/2
        n = expected['rotation'][:, 2]
        plane = expected['xyz'] @ n + expected['size'][2]/2
        error = abs(center @ n + half[2] - plane)
        assert error < 1e-7 and np.linalg.norm(frame[:, 2]-n) < 1e-7
        for other in tip.findall('collision'):
            if other is c:
                continue
            oc, of = pose(other)
            b = other.find('geometry/box'); cy = other.find('geometry/cylinder')
            if b is not None:
                support = np.abs(n @ of) @ (np.fromstring(b.get('size'), sep=' ')/2)
            else:
                along = float(n @ of[:, 2])
                support = float(cy.get('radius'))*math.sqrt(max(0, 1-along*along))+float(cy.get('length'))/2*abs(along)
            assert oc @ n + support <= plane + .00001, (name, other.get('name'), oc @ n + support-plane)
        report[name] = dict(plane_error_m=float(error), other_primitives_behind_plane=True)
    center, frame = pose(links['base_link'].find('collision'))
    assert np.allclose(center, [.0185, -.0803, -.005])
    assert abs(frame[:, 2] @ np.array([0, 1, 0])) > .999999
    report['base_actuator'] = dict(center_m=center.tolist(), shift_from_CMC_m=-.03165723526)
    # Confirm the scaled/rotated standalone palm coincides with the original.
    c = links['palm_collision'].find('collision'); mesh = c.find('geometry/mesh')
    palm = np.unique(read_stl(PACKAGE/'meshes/FINGER_v6_structure_palm.STL')['v'].reshape(-1, 3), axis=0)
    center, frame = pose(c)
    palm = (palm*np.fromstring(mesh.get('scale'), sep=' ')) @ frame.T + center
    base = np.unique(read_stl(PACKAGE/'meshes/base_link.STL')['v'].reshape(-1, 3), axis=0).astype(float)
    errors = []
    for start in range(0, len(palm), 80):
        delta = palm[start:start+80, None, :] - base[None, :, :]
        errors.extend(np.sqrt(np.sum(delta*delta, axis=2).min(1)))
    assert max(errors) < 0.000002
    report['palm_alignment'] = dict(max_vertex_error_m=float(max(errors)),
                                    scale=mesh.get('scale'), xyz=c.find('origin').get('xyz'),
                                    rpy=c.find('origin').get('rpy'))
    (PACKAGE/'validation/collision_report.json').write_text(json.dumps(report, indent=2))
    render(links)
    print(json.dumps(report, indent=2))
    print('PASS: thumb contact planes match and are unobstructed; base moved; palm aligned; tree/paths valid; original visual, inertia and joint elements unchanged. Full mesh coverage is intentionally not required.')


def lines(c):
    center, frame = pose(c)
    b, cy = c.find('geometry/box'), c.find('geometry/cylinder')
    segments = []
    if b is not None:
        corners = np.array(list(itertools.product([-1, 1], repeat=3)))
        points = corners*np.fromstring(b.get('size'), sep=' ')/2
        for i in range(8):
            for j in range(i):
                if np.count_nonzero(corners[i] != corners[j]) == 1:
                    segments.append(points[[i, j]])
    else:
        radius, length = float(cy.get('radius')), float(cy.get('length'))
        a = np.linspace(0, 2*math.pi, 65)
        for z in [-length/2, length/2]:
            ring = np.column_stack([radius*np.cos(a), radius*np.sin(a), np.full(len(a), z)])
            segments.extend([ring[i:i+2] for i in range(64)])
        for angle in np.arange(8)*math.pi/4:
            segments.append(np.array([[radius*math.cos(angle), radius*math.sin(angle), -length/2],
                                      [radius*math.cos(angle), radius*math.sin(angle), length/2]]))
    return [segment @ frame.T + center for segment in segments]


def render(links):
    names = ['base_link', 'index_proximal', 'index_intermediate', 'thumb_metacarpal', 'thumb_proximal', 'thumb_distal']
    image = Image.new('RGB', (1400, 1660), 'white'); draw = ImageDraw.Draw(image)
    draw.text((20, 12), 'Collision fit: gray = visual surface, colored outlines = collision primitives', fill='black')
    colors = ['#d44122', '#098745', '#2764c0', '#963cb0', '#b88600', '#159ea3']
    for row, name in enumerate(names):
        link = links[name]
        triangles = read_stl(PACKAGE/link.find('visual/geometry/mesh').get('filename').split('package://RAISON_v1/')[1])['v']
        points = triangles.reshape(-1, 3)
        for column, axes in enumerate([(0, 2), (0, 1)]):
            lo, hi = points[:, axes].min(0), points[:, axes].max(0)
            scale = min(600/(hi[0]-lo[0]), 200/(hi[1]-lo[1]))
            mid = (lo+hi)/2
            def project(p):
                q = (p[:, axes]-mid)*scale
                return [(int(350+column*700+x), int(160+row*270-y)) for x, y in q]
            draw.text((20+column*700, 42+row*270), name+' / '+('XZ' if column == 0 else 'XY'), fill='black')
            for tri in triangles[::max(1, len(triangles)//2500)]:
                poly = project(tri)
                draw.line(poly+[poly[0]], fill='#c4c4c4', width=1)
            for i, c in enumerate(link.findall('collision')):
                for segment in lines(c):
                    draw.line(project(segment), fill=colors[i % len(colors)], width=2)
    image.save(PACKAGE/'validation/collision_overlay.png')


if __name__ == '__main__':
    main()
