#!/usr/bin/env python3
"""Generate the screwdriver-bit tray, bit models and the g1_table world.

Writes (relative to the package root):
  models/bit_tray/                  tray with a 3 x 5 grid of square holes
  models/screwdriver_bit_<type>/    one model per bit type (hex shank + tip mesh)
  worlds/g1_table.sdf               table, tray (one hole left empty) and one loose bit

Bits are ~2x real size (12 mm hex shank instead of 1/4") so the Dex1-1 gripper,
which closes down to a ~6 mm gap, can hold them.

Re-run after editing: python3 scripts/generate_scene_assets.py
"""

import math
import os
import struct

import numpy as np

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(PKG_DIR, 'models')
WORLD_PATH = os.path.join(PKG_DIR, 'worlds', 'g1_table.sdf')

# ---------------------------------------------------------------- dimensions (m)
TABLE_SIZE = (0.6, 1.2)          # depth (x), width (y)
TABLE_HEIGHT = 0.75
TABLE_FRONT_X = 0.15             # G1 hips reach x = 0.076 at table height
TABLE_TOP_THICKNESS = 0.04

BIT_AF = 0.012                   # hex shank across flats
BIT_R = BIT_AF / math.sqrt(3)    # hex circumradius
SHANK_LEN = 0.036
NECK_TOP = 0.041
BIT_LEN = 0.050
NECK_RADIUS = 0.0045
BIT_MASS = 0.042                 # steel

TRAY_ROWS, TRAY_COLS = 3, 5      # rows along x (row 0 nearest the robot), cols along y
TRAY_PITCH = 0.034               # wide enough for the Dex1 fingers between bits
TRAY_HOLE = 0.016                # square hole; fits the hex shank in any rotation
TRAY_BORDER = 0.004
TRAY_HEIGHT = 0.030
TRAY_BASE = 0.006
TRAY_MASS = 0.3
# Static keeps the tray fixed on the table. A dynamic tray couples all bits into one contact
# problem in DART and drops the sim to ~5% real time.
TRAY_STATIC = True
TRAY_CENTER = (0.37, -0.06)      # world x, y

# Tray layout, [row][col] with col 0 at -y (robot's right). None = empty hole.
TRAY_LAYOUT = [
    ['sq1', 'ph3', None, 'ph1', 'ph0'],
    ['h2', 'sq2', 'sl5', 'sl4', 'sl3'],
    ['t20', 't15', 't10', 'h4', 'h3'],
]
LOOSE_BIT = 'ph2'                # belongs in the empty hole
LOOSE_BIT_CENTER = (0.34, 0.10)  # world x, y; lying flat, tip pointing +x
LOOSE_BIT_YAW = 0.25


# ------------------------------------------------------------------ mesh helpers
def regular_polygon(radius, n, phase=0.0):
    a = phase + 2 * np.pi * np.arange(n) / n
    return np.stack([radius * np.cos(a), radius * np.sin(a)], axis=1)


def cross_polygon(half_len, width):
    L, w = half_len, width / 2
    return np.array([[L, -w], [L, w], [w, w], [w, L], [-w, L], [-w, w], [-L, w], [-L, -w],
                     [-w, -w], [-w, -L], [w, -L], [w, -w]])


def torx_polygon(r_out, r_in, samples=72):
    a = 2 * np.pi * np.arange(samples) / samples
    r = (r_out + r_in) / 2 + (r_out - r_in) / 2 * np.cos(6 * a)
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def rect_polygon(sx, sy):
    return np.array([[sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2], [-sx / 2, -sy / 2]])


def loft(poly, z0, z1, scale0=(1.0, 1.0), scale1=(1.0, 1.0)):
    """Closed solid between a CCW, origin-star-shaped polygon scaled at z0 and at z1."""
    n = len(poly)
    bot = np.column_stack([poly * scale0, np.full(n, z0)])
    top = np.column_stack([poly * scale1, np.full(n, z1)])
    cb, ct = np.array([0, 0, z0]), np.array([0, 0, z1])
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(bot[i], bot[j], top[j]), (bot[i], top[j], top[i]),
                 (ct, top[i], top[j]), (cb, bot[j], bot[i])]
    return tris


def write_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'g1_gazebo generated'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


# Tip = (polygon, scale at tip base, scale at tip end), spanning z NECK_TOP..BIT_LEN.
TIPS = {
    'ph0': (cross_polygon(0.0025, 0.0012), (1, 1), (0.25, 0.25)),
    'ph1': (cross_polygon(0.0033, 0.0016), (1, 1), (0.25, 0.25)),
    'ph2': (cross_polygon(0.0040, 0.0019), (1, 1), (0.25, 0.25)),
    'ph3': (cross_polygon(0.0045, 0.0023), (1, 1), (0.25, 0.25)),
    'sl3': (rect_polygon(0.0020, 0.0060), (1, 1), (0.35, 1)),
    'sl4': (rect_polygon(0.0022, 0.0075), (1, 1), (0.35, 1)),
    'sl5': (rect_polygon(0.0024, 0.0090), (1, 1), (0.35, 1)),
    'h2': (regular_polygon(0.004 / math.sqrt(3), 6, math.pi / 6), (1, 1), (1, 1)),
    'h3': (regular_polygon(0.005 / math.sqrt(3), 6, math.pi / 6), (1, 1), (1, 1)),
    'h4': (regular_polygon(0.006 / math.sqrt(3), 6, math.pi / 6), (1, 1), (1, 1)),
    't10': (torx_polygon(0.0028, 0.0020), (1, 1), (1, 1)),
    't15': (torx_polygon(0.0033, 0.0024), (1, 1), (1, 1)),
    't20': (torx_polygon(0.0039, 0.0028), (1, 1), (1, 1)),
    'sq1': (rect_polygon(0.0035, 0.0035), (1, 1), (0.8, 0.8)),
    'sq2': (rect_polygon(0.0045, 0.0045), (1, 1), (0.8, 0.8)),
}


def bit_mesh(tip):
    poly, s0, s1 = TIPS[tip]
    # Hex flats face local +-x, so a bit lying on its side rests on a flat.
    return (loft(regular_polygon(BIT_R, 6, math.pi / 6), 0.0, SHANK_LEN)
            + loft(regular_polygon(NECK_RADIUS, 24), SHANK_LEN, NECK_TOP)
            + loft(poly, NECK_TOP, BIT_LEN, s0, s1))


# ------------------------------------------------------------------- SDF helpers
def box_inertia(m, x, y, z):
    return m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12


def inertial_xml(m, ixx, iyy, izz, z, indent):
    p = ' ' * indent
    return (f'{p}<inertial>\n{p}  <pose>0 0 {z:.4f} 0 0 0</pose>\n{p}  <mass>{m}</mass>\n'
            f'{p}  <inertia><ixx>{ixx:.3e}</ixx><iyy>{iyy:.3e}</iyy><izz>{izz:.3e}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>\n{p}</inertial>\n')


def model_config(name, description):
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            f'  <sdf version="1.9">model.sdf</sdf>\n  <description>{description}</description>\n'
            f'</model>\n')


def write_model(name, description, sdf):
    d = os.path.join(MODELS_DIR, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'model.config'), 'w') as f:
        f.write(model_config(name, description))
    with open(os.path.join(d, 'model.sdf'), 'w') as f:
        f.write(sdf)
    return d


def bit_model(tip):
    name = f'screwdriver_bit_{tip}'
    d = os.path.join(MODELS_DIR, name, 'meshes')
    os.makedirs(d, exist_ok=True)
    write_stl(os.path.join(d, f'bit_{tip}.stl'), bit_mesh(tip))

    friction = '<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>'
    # Union of three boxes rotated by 60 deg is exactly the hex prism.
    shank = ''.join(
        f'      <collision name="shank_{k}">\n'
        f'        <pose>0 0 {SHANK_LEN / 2} 0 0 {k * math.pi / 3:.6f}</pose>\n'
        f'        <geometry><box><size>{BIT_AF} {BIT_R:.6f} {SHANK_LEN}</size></box></geometry>\n'
        f'        {friction}\n      </collision>\n' for k in range(3))
    tip_len = BIT_LEN - SHANK_LEN
    r2 = BIT_R ** 2
    ixx = BIT_MASS * (3 * r2 + BIT_LEN ** 2) / 12
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
{inertial_xml(BIT_MASS, ixx, ixx, BIT_MASS * r2 / 2, BIT_LEN / 2, 6)}{shank}      <collision name="tip">
        <pose>0 0 {SHANK_LEN + tip_len / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{NECK_RADIUS}</radius><length>{tip_len}</length></cylinder></geometry>
        {friction}
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{name}/meshes/bit_{tip}.stl</uri></mesh></geometry>
        <material>
          <ambient>0.45 0.46 0.48 1</ambient>
          <diffuse>0.62 0.63 0.66 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''
    write_model(name, f'{tip.upper()} screwdriver bit, 12 mm hex shank, {BIT_LEN * 1000:.0f} mm long '
                      '(2x scale). Origin at the shank end, +z towards the tip.', sdf)


def tray_geometry():
    """Boxes (center xyz, size xyz) forming the tray; origin at the bottom center."""
    X = TRAY_ROWS * TRAY_PITCH + 2 * TRAY_BORDER
    Y = TRAY_COLS * TRAY_PITCH + 2 * TRAY_BORDER
    xs = [-X / 2 + TRAY_BORDER + TRAY_PITCH / 2 + i * TRAY_PITCH for i in range(TRAY_ROWS)]
    ys = [-Y / 2 + TRAY_BORDER + TRAY_PITCH / 2 + j * TRAY_PITCH for j in range(TRAY_COLS)]
    h, wall_h = TRAY_HOLE / 2, TRAY_HEIGHT - TRAY_BASE
    zc = TRAY_BASE + wall_h / 2
    boxes = [((0, 0, TRAY_BASE / 2), (X, Y, TRAY_BASE))]
    x_edges = [-X / 2] + [e for x in xs for e in (x - h, x + h)] + [X / 2]
    for a, b in zip(x_edges[0::2], x_edges[1::2]):          # full-width strips between rows
        boxes.append((((a + b) / 2, 0, zc), (b - a, Y, wall_h)))
    y_edges = [-Y / 2] + [e for y in ys for e in (y - h, y + h)] + [Y / 2]
    for x in xs:                                            # blocks between holes in a row
        for a, b in zip(y_edges[0::2], y_edges[1::2]):
            boxes.append(((x, (a + b) / 2, zc), (TRAY_HOLE, b - a, wall_h)))
    return X, Y, xs, ys, boxes


def tray_model():
    X, Y, _, _, boxes = tray_geometry()
    parts = []
    for k, ((cx, cy, cz), (sx, sy, sz)) in enumerate(boxes):
        geom = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
        pose = f'<pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>'
        parts.append(
            f'      <collision name="c{k}">{pose}{geom}\n'
            f'        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>\n'
            f'      </collision>\n'
            f'      <visual name="v{k}">{pose}{geom}\n'
            f'        <material><ambient>0.72 0.74 0.78 1</ambient><diffuse>0.86 0.88 0.92 1</diffuse>'
            f'<specular>0.3 0.3 0.3 1</specular></material>\n'
            f'      </visual>\n')
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="bit_tray">
    <static>{str(TRAY_STATIC).lower()}</static>
    <link name="link">
{inertial_xml(TRAY_MASS, *box_inertia(TRAY_MASS, X, Y, TRAY_HEIGHT), TRAY_HEIGHT / 2, 6)}{''.join(parts)}    </link>
  </model>
</sdf>
'''
    write_model('bit_tray', f'Screwdriver bit holder, {TRAY_ROWS}x{TRAY_COLS} square holes '
                            f'({TRAY_HOLE * 1000:.0f} mm, {TRAY_PITCH * 1000:.0f} mm pitch), '
                            f'{X * 1000:.0f} x {Y * 1000:.0f} x {TRAY_HEIGHT * 1000:.0f} mm. '
                            'Origin at the bottom center.', sdf)


# ------------------------------------------------------------------------- world
def include(uri, name, pose):
    return (f'    <include>\n      <uri>model://{uri}</uri>\n      <name>{name}</name>\n'
            f'      <pose>{" ".join(f"{v:.4f}" for v in pose)}</pose>\n    </include>\n')


def table_model():
    dx, dy = TABLE_SIZE
    cx = TABLE_FRONT_X + dx / 2
    top_z = TABLE_HEIGHT - TABLE_TOP_THICKNESS / 2
    leg_h = TABLE_HEIGHT - TABLE_TOP_THICKNESS
    leg_mat = '<material><ambient>0.3 0.2 0.12 1</ambient><diffuse>0.35 0.23 0.14 1</diffuse></material>'
    legs = ''
    for name, sx, sy in (('fl', 1, 1), ('fr', 1, -1), ('bl', -1, 1), ('br', -1, -1)):
        pose = f'<pose>{sx * (dx / 2 - 0.05):.3f} {sy * (dy / 2 - 0.05):.3f} {leg_h / 2:.3f} 0 0 0</pose>'
        geom = f'<geometry><box><size>0.05 0.05 {leg_h:.3f}</size></box></geometry>'
        legs += (f'        <collision name="leg_{name}_collision">{pose}{geom}</collision>\n'
                 f'        <visual name="leg_{name}_visual">{pose}{geom}{leg_mat}</visual>\n')
    top_geom = f'<geometry><box><size>{dx} {dy} {TABLE_TOP_THICKNESS}</size></box></geometry>'
    return f'''    <!-- Table: {dx} m deep (x) x {dy} m wide (y), surface at z = {TABLE_HEIGHT} m, front edge at x = {TABLE_FRONT_X} m. -->
    <model name="table">
      <static>true</static>
      <pose>{cx:.3f} 0 0 0 0 0</pose>
      <link name="link">
        <collision name="top_collision">
          <pose>0 0 {top_z:.3f} 0 0 0</pose>
          {top_geom}
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="top_visual">
          <pose>0 0 {top_z:.3f} 0 0 0</pose>
          {top_geom}
          <material><ambient>0.45 0.3 0.18 1</ambient><diffuse>0.55 0.36 0.2 1</diffuse></material>
        </visual>
{legs}      </link>
    </model>
'''


def world_sdf():
    _, _, xs, ys, _ = tray_geometry()
    tx, ty = TRAY_CENTER
    objects = include('bit_tray', 'bit_tray', (tx, ty, TABLE_HEIGHT + 0.0005, 0, 0, 0))
    bit_z = TABLE_HEIGHT + TRAY_BASE + 0.001
    for i, row in enumerate(TRAY_LAYOUT):
        for j, tip in enumerate(row):
            if tip is not None:
                objects += include(f'screwdriver_bit_{tip}', f'bit_{tip}',
                                   (tx + xs[i], ty + ys[j], bit_z, 0, 0, 0))
    # Lying on a hex flat: pitch +90 deg turns the bit axis (+z) to +x.
    lx, ly = LOOSE_BIT_CENTER
    ox = lx - BIT_LEN / 2 * math.cos(LOOSE_BIT_YAW)
    oy = ly - BIT_LEN / 2 * math.sin(LOOSE_BIT_YAW)
    objects += include(f'screwdriver_bit_{LOOSE_BIT}', f'bit_{LOOSE_BIT}_loose',
                       (ox, oy, TABLE_HEIGHT + BIT_AF / 2 + 0.0005, 0, math.pi / 2, LOOSE_BIT_YAW))
    empty = [(i, j) for i, row in enumerate(TRAY_LAYOUT) for j, t in enumerate(row) if t is None]
    empty_xy = ', '.join(f'({tx + xs[i]:.3f}, {ty + ys[j]:.3f})' for i, j in empty)

    return f'''<?xml version="1.0"?>
<!-- Generated by scripts/generate_scene_assets.py - edit that script and re-run it. -->
<sdf version="1.9">
  <world name="g1_table_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{table_model()}
    <!-- Bit tray with one empty hole at {empty_xy}; the matching {LOOSE_BIT.upper()} bit lies on the table. -->
{objects}  </world>
</sdf>
'''


def main():
    for tip in TIPS:
        bit_model(tip)
    tray_model()
    with open(WORLD_PATH, 'w') as f:
        f.write(world_sdf())
    print(f'Wrote {len(TIPS)} bit models + bit_tray to {MODELS_DIR} and {WORLD_PATH}')


if __name__ == '__main__':
    main()
