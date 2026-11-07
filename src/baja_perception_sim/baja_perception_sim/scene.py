"""Generate self-contained SDF + meshes + exact scene manifest, without ROS."""
import argparse
import json
import math
from pathlib import Path
import random
from xml.sax.saxutils import escape

import numpy as np
from scipy.spatial import cKDTree
import yaml

from .geometry import track_length, track_pose, track_samples

BLUE = '0.03 0.15 0.95 1'
RED = '0.95 0.04 0.03 1'


def mesh(path, vertices, faces):
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    normals = np.zeros_like(vertices)
    a,b,c = vertices[faces[:,0]],vertices[faces[:,1]],vertices[faces[:,2]]
    face_normals = np.cross(b-a,c-a)
    for column in range(3): np.add.at(normals,faces[:,column],face_normals)
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
    path.with_suffix('.mtl').write_text('newmtl default\nKd 1 1 1\nKa 1 1 1\n')
    with path.open('w') as f:
        f.write(f'mtllib {path.stem}.mtl\no surface\nusemtl default\n')
        for p in vertices:
            f.write('v ' + ' '.join(f'{v:.6f}' for v in p) + '\n')
        for normal in normals:
            f.write('vn ' + ' '.join(f'{v:.6f}' for v in normal) + '\n')
        for face in faces:
            f.write('f ' + ' '.join(f'{i+1}//{i+1}' for i in face) + '\n')


def cone_mesh(path):
    n = 24
    v = [(0.5*math.cos(a), 0.5*math.sin(a), 0) for a in np.linspace(0, 2*math.pi, n, endpoint=False)]
    v += [(0, 0, 1), (0, 0, 0)]
    faces = [(i, (i+1) % n, n) for i in range(n)]
    faces += [(n+1, (i+1) % n, i) for i in range(n)]
    mesh(path, v, faces)


def height(x, y, cfg):
    t = cfg['terrain']
    if not t['enabled'] or abs(y) > cfg['track']['width'] / 2:
        return 0.0
    start, length = t['sine_start'], t['sine_length']
    if start <= x <= start + length:
        u = (x-start)/length
        return t['sine_peak_to_peak']/2 * math.sin(2*math.pi*t['sine_cycles']*u) * math.sin(math.pi*u)**2
    # Nine smoothly tapered depressions, real mesh recesses rather than decals.
    r = t['crater_radius']
    for cx in [18, 18 + 2*r + 0.05, 18 + 4*r + 0.10]:
        for cy in [-(2*r+0.03), 0, 2*r+0.03]:
            d = math.hypot(x-cx, y-cy) / r
            if d < 1:
                return -t['crater_depth'] * (1-d*d)**2
    # Smooth semi-elliptical bump across the road.
    u = (x-26) / 0.30
    if abs(u) < 1:
        return t['bump_height'] * math.sqrt(1-u*u)
    return 0.0


def geometry(kind, size, cone_uri=None):
    if kind == 'cone':
        return f'<mesh><uri>{escape(cone_uri)}</uri><scale>{size[0]} {size[1]} {size[2]}</scale></mesh>'
    if kind == 'barrel':
        return f'<cylinder><radius>{size[0]/2}</radius><length>{size[2]}</length></cylinder>'
    return '<box><size>' + ' '.join(map(str, size)) + '</size></box>'


def component(name, geom, color, pose='0 0 0 0 0 0', collide=True):
    visual = f'<visual name="{name}"><pose>{pose}</pose><geometry>{geom}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'
    collision = f'<collision name="{name}"><pose>{pose}</pose><geometry>{geom}</geometry></collision>' if collide else ''
    return visual + collision


def static_model(name, body, pose='0 0 0 0 0 0'):
    return f'<model name="{name}"><static>true</static><pose>{pose}</pose><link name="body">{body}</link></model>'


def object_model(obj, cone_uri):
    x, y, z = obj['position']
    w, d, h = obj['size']
    kind = obj['class_id']
    if kind == 'cone':
        body = component('cone', geometry(kind, obj['size'], cone_uri), '1 0.45 0.03 1', f'0 0 {-h/2} 0 0 0')
    elif kind == 'person':
        body = component('torso', geometry('box', [w, d, h*0.52]), '0.15 0.35 0.8 1', f'0 0 {h*0.08} 0 0 0')
        body += component('head', f'<sphere><radius>{w*0.29}</radius></sphere>', '0.7 0.5 0.35 1', f'0 0 {h/2-w*0.29} 0 0 0')
        for side in [-1, 1]:
            body += component(f'leg_{side}', geometry('box', [w*0.35, d*0.35, h*0.40]), '0.12 0.12 0.15 1', f'0 {side*d*0.25} {-h*0.30} 0 0 0')
    else:
        body = component('body', geometry(kind, obj['size']), '0.6 0.3 0.08 1' if kind == 'box' else '0.1 0.65 0.4 1')
    return static_model(obj['id'], body, f'{x} {y} {z} 0 0 {obj["yaw"]}')


def generate(cfg, output, vehicle_template):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    assets = output / 'meshes'
    assets.mkdir(exist_ok=True)
    cone_mesh(assets / 'cone.obj')
    cone_uri = (assets / 'cone.obj').as_uri()
    tc = cfg['track']
    if min(tc['straight_x'], tc['straight_y'], tc['sample_step'], tc['cone_height'], tc['cone_base_width']) <= 0:
        raise ValueError('Track dimensions, sample step and cone dimensions must be positive')
    if cfg['obstacles']['count'] < 0 or not cfg['obstacles']['classes']:
        raise ValueError('Obstacle count must be nonnegative and classes must not be empty')
    speed = cfg['vehicle']['speed']
    max_speed = cfg['vehicle'].get('max_speed', 3.0)
    if not math.isfinite(max_speed) or not 0 < max_speed <= 5.0:
        raise ValueError('vehicle.max_speed must be in (0,5] m/s for this vehicle')
    if not math.isfinite(speed) or not 0 < speed <= max_speed:
        raise ValueError(f'vehicle.speed must be in (0,{max_speed}] m/s')
    if cfg['terrain']['sine_length'] <= 0 or cfg['terrain']['crater_radius'] <= 0:
        raise ValueError('Terrain length and crater radius must be positive')
    if cfg['lidar']['channels'] != 16 or cfg['lidar']['columns'] != 2000:
        raise ValueError('Interface v0.1 expects 16 channels x 2000 columns')
    if tc['width']/2-0.6 < 3.4:
        raise ValueError('Default side obstacle bands need road width >=8m')
    if tc['radius'] <= tc['width']/2 + 1:
        raise ValueError('Turn radius must exceed half road width + 1m')
    if tc['cone_spacing'] <= tc['cone_base_width']:
        raise ValueError('Cone spacing must exceed cone base width')
    if cfg['vehicle']['clear_radius'] < math.hypot(cfg['vehicle']['length'], cfg['vehicle']['width'])/2 + 0.5:
        raise ValueError('clear_radius must cover vehicle half diagonal + >=0.5m tracking tolerance')
    path = track_samples(tc)
    tree = cKDTree(path[:, :2])
    # Shared ground mesh at sufficient resolution for the small terrain features.
    lateral = np.linspace(-tc['width']/2, tc['width']/2, math.ceil(tc['width']/0.2)+1)
    vertices = []
    for x, y, yaw, _ in path:
        for d in lateral:
            px, py = x-math.sin(yaw)*d, y+math.cos(yaw)*d
            vertices.append((px, py, height(px, py, cfg)))
    faces = []
    m, n = len(lateral), len(path)
    for i in range(n):
        for j in range(m-1):
            a, b = i*m+j, ((i+1) % n)*m+j
            faces += [(a, b, b+1), (a, b+1, a+1)]
    mesh(assets / 'road.obj', vertices, faces)
    parts = [static_model('road', component('surface', f'<mesh><uri>{(assets / "road.obj").as_uri()}</uri></mesh>', '0.30 0.31 0.33 1'))]
    parts.append(static_model('surroundings', component('ground', '<box><size>180 150 0.1</size></box>', '0.30 0.40 0.20 1'), '15 17 -0.55 0 0 0'))
    boundaries = {'left': [], 'right': []}
    cone_parts = []
    count = math.ceil(track_length(tc)/tc['cone_spacing'])
    for i, s in enumerate(np.linspace(0, track_length(tc), count, endpoint=False)):
        x, y, yaw = track_pose(s, tc)
        for side, sign, color in [('left', 1, BLUE), ('right', -1, RED)]:
            px = x-sign*tc['width']/2*math.sin(yaw)
            py = y+sign*tc['width']/2*math.cos(yaw)
            z = height(px, py, cfg)
            boundaries[side].append([px, py, z])
            geom = geometry('cone', [tc['cone_base_width']]*2+[tc['cone_height']], cone_uri)
            cone_parts.append(component(f'{side}_{i}', geom, color, f'{px} {py} {z} 0 0 0'))
    parts.append(static_model('boundary_cones', ''.join(cone_parts)))
    rng = random.Random(cfg['seed'])
    objects = []
    attempts = 0
    while len(objects) < cfg['obstacles']['count']:
        attempts += 1
        if attempts > 20000:
            raise ValueError('Cannot place all obstacles safely; reduce count or dimensions')
        # A few initial targets are visible soon after starting.
        s = rng.uniform(7, 29) if len(objects) < min(6, cfg['obstacles']['count']) else rng.uniform(0, track_length(tc))
        d = rng.choice([-1, 1]) * rng.uniform(3.4, tc['width']/2-0.6)
        x, y, yaw = track_pose(s, tc)
        x, y = x-d*math.sin(yaw), y+d*math.cos(yaw)
        kind = rng.choice(cfg['obstacles']['classes'])
        if kind not in ['box', 'barrel', 'cone', 'person']:
            raise ValueError(f'Unknown obstacle class {kind}')
        size = {'box': [rng.uniform(0.4, 0.9), rng.uniform(0.4, 0.8), rng.uniform(0.35, 1.0)],
                'barrel': [0.55, 0.55, rng.uniform(0.65, 0.95)],
                'cone': [0.30, 0.30, rng.uniform(0.3, 0.5)],
                'person': [0.55, 0.5, 1.7]}[kind]
        radius = math.hypot(size[0], size[1])/2
        # Sampling correction makes the complete-curve clearance conservative.
        clearance = float(tree.query([x, y])[0]) - tc['sample_step']/2 - radius
        if clearance < cfg['vehicle']['clear_radius'] or abs(d)+radius > tc['width']/2-0.25:
            continue
        if any(math.hypot(x-o['position'][0], y-o['position'][1]) < radius+math.hypot(*o['size'][:2])/2+cfg['obstacles']['min_gap'] for o in objects):
            continue
        obj = {'id': f'object_{len(objects):03}', 'class_id': kind,
               'position': [x, y, height(x, y, cfg)+size[2]/2],
               'size': size, 'yaw': rng.uniform(-math.pi, math.pi), 'clearance': clearance}
        objects.append(obj)
        parts.append(object_model(obj, cone_uri))
    regions = []
    if cfg['terrain']['enabled']:
        for kind, a, b, h in [('sine_wave', cfg['terrain']['sine_start'], cfg['terrain']['sine_start']+cfg['terrain']['sine_length'], cfg['terrain']['sine_peak_to_peak']),
                               ('crater', 17.3, 22.1, -cfg['terrain']['crater_depth']),
                               ('speed_bump', 25.7, 26.3, cfg['terrain']['bump_height'])]:
            regions.append({'class_id': kind, 'height': h, 'polygon': [[a,-3,0],[b,-3,0],[b,3,0],[a,3,0]]})
    # Zebra crossing on the east straight; purely visual, no raised collisions.
    for i in range(7):
        parts.append(static_model(f'zebra_{i}', component('paint', geometry('box', [10, 0.25, 0.003]), '0.95 0.95 0.9 1', collide=False), f'40 {15+i*0.5} 0.003 0 0 0'))
    regions.append({'class_id':'crosswalk','height':0.0,'polygon':[[34,14.8,0],[46,14.8,0],[46,18.3,0],[34,18.3,0]]})
    # Parking bay protrudes beyond the west road edge, away from the driving corridor.
    park = [[-16,16,0],[-19,16,0],[-19,22,0],[-16,22,0]]
    regions.append({'class_id':'parking_bay','height':0.0,'polygon':park})
    for i, (a,b) in enumerate(zip(park, park[1:]+park[:1])):
        length = math.dist(a,b)
        for j in range(math.ceil(length/0.5)):
            p = np.array(a)+(np.array(b)-a)*j/math.ceil(length/0.5)
            parts.append(static_model(f'parking_{i}_{j}', component('cone', geometry('cone',[0.24,0.24,0.3],cone_uri), RED), f'{p[0]} {p[1]} 0 0 0 0'))
    # Traffic fixtures outside the road: state encoded as explicit class labels.
    for kind, x, y, yaw in [('traffic_light_red',33,16,math.pi/2),('speed_limit_20',47,23,math.pi/2)]:
        body = component('pole', '<cylinder><radius>0.05</radius><length>2.4</length></cylinder>', '0.4 0.4 0.4 1', '0 0 1.2 0 0 0')
        if kind.startswith('traffic'):
            body += component('housing', geometry('box',[0.2,0.4,0.95]), '0.05 0.05 0.05 1', '0 0 2.4 0 0 0')
            for j, color in enumerate([RED, '0.18 0.16 0.01 1', '0.01 0.12 0.01 1']):
                body += component(f'light_{j}', '<sphere><radius>0.12</radius></sphere>', color, f'-0.12 0 {2.7-j*0.3} 0 0 0', False)
            size = [0.5,0.5,2.9]
        else:
            body += component('rim','<cylinder><radius>0.48</radius><length>0.08</length></cylinder>',RED,'0 0 2.5 0 1.5707963 0')
            body += component('face','<cylinder><radius>0.40</radius><length>0.01</length></cylinder>','1 1 1 1','-0.045 0 2.5 0 1.5707963 0',False)
            seven = {'2':[0,1,3,4,6], '0':[0,1,2,3,4,5]}
            segments = [(0,.2,.14,.03),(-.075,.1,.03,.18),(-.075,-.1,.03,.18),(0,-.2,.14,.03),(.075,-.1,.03,.18),(.075,.1,.03,.18),(0,0,.14,.03)]
            for digit, offset in [('2',.14),('0',-.14)]:
                for k in seven[digit]:
                    dy,dz,wy,hz=segments[k]
                    body += component(f'digit_{digit}_{k}',geometry('box',[.01,wy,hz]),'0.01 0.01 0.01 1',f'-0.055 {offset+dy} {2.5+dz} 0 0 0',False)
            size = [0.15,0.96,2.98]
        obj = {'id':kind,'class_id':kind,'position':[x,y,size[2]/2],'size':size,'yaw':yaw}
        objects.append(obj)
        parts.append(static_model(kind,body,f'{x} {y} 0 0 0 {yaw}'))
    # Independently check every scored fixture against the full path as well.
    for o in objects:
        clearance = float(tree.query(o['position'][:2])[0])-tc['sample_step']/2-math.hypot(*o['size'][:2])/2
        if clearance < cfg['vehicle']['clear_radius']:
            raise ValueError(f'Unsafe object {o["id"]}: {clearance:.3f}m')
        o['clearance'] = clearance
    vehicle = render_vehicle(cfg, vehicle_template)
    parts.append(vehicle)
    world = '''<?xml version="1.0"?><sdf version="1.9"><world name="perception_track">
      <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <gravity>0 0 -9.81</gravity><scene><ambient>0.6 0.6 0.6 1</ambient><background>0.65 0.8 0.95 1</background><shadows>false</shadows></scene>
      <light name="sun" type="directional"><pose>0 0 50 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 -0.4 -1</direction><cast_shadows>false</cast_shadows></light>
    ''' + '\n'.join(parts) + '</world></sdf>'
    (output / 'world.sdf').write_text(world)
    manifest = {'schema_version':1,'config':cfg,'length':track_length(tc),'path':path.tolist(),
                'boundaries':boundaries,'objects':objects,'regions':regions,
                'cone_count':2*count, 'lidar_mount':[0,0,cfg['lidar']['ground_height']-0.457],
                'camera_mount':[1.25,0,cfg['camera']['ground_height']-0.457]}
    (output / 'scene.json').write_text(json.dumps(manifest,indent=2))
    return manifest


def render_vehicle(cfg, template):
    import xml.etree.ElementTree as ET
    model = ET.parse(template).getroot().find('model')
    model.find('pose').text = '0 0 0.46 0 0 0'
    model.find("plugin[@name='gz::sim::systems::AckermannSteering']/max_velocity").text = str(cfg['vehicle'].get('max_speed', 3.0))
    sensor = model.find("link[@name='base_link']/sensor[@name='lidar']")
    sensor.find('pose').text = f'0 0 {cfg["lidar"]["ground_height"]-0.457} 0 0 0'
    lc = cfg['lidar']
    for tag,value in [('update_rate',lc['rate']),('lidar/scan/horizontal/samples',lc['columns']),
                      ('lidar/scan/horizontal/max_angle',math.pi-2*math.pi/lc['columns']),
                      ('lidar/scan/vertical/samples',lc['channels']),
                      ('lidar/scan/vertical/min_angle',math.radians(lc['vertical_min_deg'])),
                      ('lidar/scan/vertical/max_angle',math.radians(lc['vertical_max_deg'])),
                      ('lidar/range/min',lc['min_range']),('lidar/range/max',lc['max_range']),
                      ('lidar/noise/stddev',lc['noise_stddev'])]:
        sensor.find(tag).text = str(value)
    camera = model.find("link[@name='base_link']/sensor[@name='front_camera']")
    cc = cfg['camera']
    camera.find('pose').text=f'1.25 0 {cc["ground_height"]-0.457} 0 0 0'
    for tag,value in [('update_rate',cc['rate']),('camera/image/width',cc['width']),('camera/image/height',cc['height']),('camera/horizontal_fov',math.radians(cc['horizontal_fov_deg']))]:
        camera.find(tag).text=str(value)
    return ET.tostring(model,encoding='unicode')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--vehicle',required=True)
    parser.add_argument('--seed',type=int)
    parser.add_argument('--obstacles',type=int)
    args=parser.parse_args()
    cfg=yaml.safe_load(Path(args.config).read_text())
    if args.seed is not None: cfg['seed']=args.seed
    if args.obstacles is not None: cfg['obstacles']['count']=args.obstacles
    m=generate(cfg,args.output,args.vehicle)
    print(f'Generated {m["length"]:.2f}m loop, {m["cone_count"]} boundary cones, {len(m["objects"])} objects in {args.output}')


if __name__ == '__main__': main()
