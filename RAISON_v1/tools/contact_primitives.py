"""Main-body approximations; intentionally omit small hardware protrusions.

Analytic hinges, motors and connecting webs replace enclosing bounding boxes.
Thumb contact planes are measured from the original STL, without padding.
All dimensions below are meters. Collision geometry does not change inertia.
"""
import math
import numpy as np
from fit_collision_primitives import partition, subdivide

H = np.array([math.cos(math.pi/9), math.sin(math.pi/9), 0.0])
L = np.array([math.sin(math.pi/9), -math.cos(math.pi/9), 0.0])


def frame_for_axis(axis):
    z = np.asarray(axis, float); z /= np.linalg.norm(z)
    ref = np.eye(3)[np.argmin(np.abs(z))]
    x = np.cross(ref, z); x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def cylinder(name, center, axis, radius, length):
    return dict(name=name, shape='cylinder', xyz=np.asarray(center),
                rotation=frame_for_axis(axis), radius=radius, length=length)


def box(name, center, size, rotation=None):
    return dict(name=name, shape='box', xyz=np.asarray(center),
                rotation=np.eye(3) if rotation is None else rotation, size=np.asarray(size))


def web(name, a, b, axis, width, depth):
    a, b, axis = np.asarray(a), np.asarray(b), np.asarray(axis)
    along = b-a; length = np.linalg.norm(along); along /= length
    across = np.cross(axis, along); across /= np.linalg.norm(across)
    rotation = np.column_stack([across, axis, along])
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1
    return box(name, (a+b)/2, [width, depth, length], rotation)


def components(triangles):
    verts, inv = np.unique(np.round(triangles.reshape(-1, 3), 7), axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3); parent = list(range(len(verts)))
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    for a, b, c in faces:
        root = find(a); parent[find(b)] = root; parent[find(c)] = root
    labels = np.array([find(a) for a in faces[:, 0]])
    return [triangles[labels == label] for label in np.unique(labels)]


def area(triangles):
    return np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                   triangles[:, 2]-triangles[:, 0]), axis=1).sum()/2


def approximate_structure(triangles, count=3):
    result = []
    for i, (region, proxy) in enumerate(partition(triangles, count, return_regions=True)):
        q = region.reshape(-1, 3) @ proxy['rotation']
        lo, hi = np.quantile(q, [0.025, 0.975], axis=0)
        center = (lo+hi)/2
        proxy['xyz'] = center @ proxy['rotation'].T
        proxy['name'] = f'structure_{i+1}'
        if proxy['shape'] == 'box':
            proxy['size'] = np.maximum(hi-lo, 0.001)
        else:
            proxy['radius'] = float(np.quantile(np.linalg.norm((q-center)[:, :2], axis=1), 0.95))
            proxy['length'] = float(hi[2]-lo[2])
        result.append(proxy)
    return result


def thumb_contact_plane(triangles, target, name):
    cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    twice_area = np.linalg.norm(cross, axis=1)
    normals = cross/np.maximum(twice_area[:, None], 1e-20)
    target = np.asarray(target); target /= np.linalg.norm(target)
    candidates = np.where(normals @ target > 0.99999)[0]
    primary = candidates[np.argmax(twice_area[candidates])]
    normal = normals[primary]; distance = triangles[primary, 0] @ normal
    selected = ((normals @ normal > 0.999999)
                & (np.max(np.abs(triangles @ normal-distance), axis=1) < 0.000002))
    x = H - normal*(H @ normal); x /= np.linalg.norm(x)
    rotation = np.column_stack([x, np.cross(normal, x), normal])
    q = triangles[selected].reshape(-1, 3) @ rotation
    lo, hi = q.min(0), q.max(0)
    # Inset the tangential edges by 0.5 mm, but keep the exterior plane exact.
    center = (lo+hi)/2; center[2] = distance-0.001
    size = hi-lo; size[:2] -= 0.001; size[2] = 0.002
    return box(name, center @ rotation.T, size, rotation)


def build(name, triangles):
    triangles = triangles.astype(float)
    points = triangles.reshape(-1, 3)
    if name == 'thumb_distal':
        return [thumb_contact_plane(triangles, [-.0885, .2432, -.9659], 'thumb_front_contact'),
                thumb_contact_plane(triangles, [.1932, -.5307, .8252], 'thumb_back_contact'),
                cylinder('thumb_IP_hinge', H*.01295, H, .015, .0025),
                cylinder('thumb_rounded_tip', H*(-.0008)+L*.057+np.array([0, 0, -.026]), H, .006, .028)]
    if name in ['thumb_proximal', 'thumb_metacarpal']:
        groups = components(triangles)
        # The common actuator drum is a disconnected 2560 mm^2 surface.
        drum = min(groups, key=lambda t: abs(area(t)-.002560))
        assert abs(area(drum)-.002560) < .00002
        rotation = frame_for_axis(H); q = drum.reshape(-1, 3) @ rotation
        center = (q.min(0)+q.max(0))/2
        motor = cylinder('actuator_body', center @ rotation.T, H, .019, .030)
        structure = subdivide(max(groups, key=area), max_edge=.003)
        local = (structure.mean(1)-motor['xyz']) @ rotation
        outside = np.linalg.norm(local[:, :2], axis=1) > .020
        return [motor] + approximate_structure(structure[outside], 3)
    if name.endswith('_proximal'):
        # Ring and index have identical bodies but different local Y origins.
        shift = float((points[:, 1].min()+points[:, 1].max())/2 - .004070333)
        yc = -.001372166 + shift
        axis = [0, 1, 0]
        return [cylinder('MCP_motor', [0, .001277833+shift, 0], axis, .0175, .024),
                cylinder('offset_motor', [-.03544, .00548+shift, .00625], axis, .0175, .023),
                cylinder('PIP_hinge', [.0065, yc, -.045], axis, .0115, .0168),
                web('phalange_web', [0, yc, -.010], [.0065, yc, -.039], axis, .014, .0168),
                web('motor_mount', [-.03544, .0125+shift, .00625], [0, .0125+shift, 0], axis, .007, .004)]
    if name.endswith('_intermediate'):
        yc = float((points[:, 1].min()+points[:, 1].max())/2)
        depth = float(np.ptp(points[:, 1])); axis = [0, 1, 0]
        a = np.array([0, yc, 0]); b = np.array([.0035, yc, -.035]); mid=(a+b)/2
        return [cylinder('PIP_hinge', a, axis, .011, depth),
                cylinder('DIP_hinge', b, axis, .0075, depth),
                web('upper_web', a, mid, axis, .018, depth),
                web('lower_web', mid, b, axis, .014, depth)]
    if name.endswith('_distal'):
        yc = float((points[:, 1].min()+points[:, 1].max())/2)
        depth = float(np.ptp(points[:, 1])); axis = [0, 1, 0]
        a = np.array([0, yc, 0]); b = np.array([.0025, yc, -.0195])
        return [cylinder('DIP_hinge', a, axis, .0075, depth),
                cylinder('rounded_tip', b, axis, .005, depth),
                web('tip_web', a, b, axis, .010, depth)]
    return approximate_structure(triangles, 3)
