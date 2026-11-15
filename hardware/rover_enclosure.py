"""LUNA-GAP rover enclosure for the Arduino UNO Q.

Parametric, prints on a Bambu (or any FDM) printer without supports.
Run:  python rover_enclosure.py      -> writes STLs into ./stl/

Parts
  body.stl   open-top chassis tub with rocker-bogie side arms, wheel pegs,
             UNO hole-pattern standoffs, rear port window, front vents/headlights,
             optional under-floor camera pocket (Raspberry Pi Cam v2 / IMX219 size)
  lid.stl    friction-fit roof with "solar panel" wings and a recessed ArUco
             marker pad (for the overhead-camera tracking). Print TOP FACE DOWN.
  mast_post.stl + mast_head.stl  white post + red camera head (head tenon glues into post).
  wheel.stl  print 6x. Press/glue onto the 4 mm pegs.

All units mm. Axes: X = length (rear x=0 -> front), Y = width, Z = up.
"""
from pathlib import Path

import numpy as np
import trimesh
from trimesh.creation import box as _box, cylinder as _cyl

# ------------------------------------------------------------------ parameters
# v2: real stack = UNO Media Carrier (bottom, tall header + 3.5 mm jack + camera FPCs on
# its UNDERSIDE) -> UNO Q -> Grove Base Shield. v1 was too tight, so: bigger side gap,
# 10 mm standoffs to clear the carrier's underside parts, and a front bay for a Grove OLED.
BOARD = (68.6, 53.4)      # UNO Q / Media Carrier / Grove shield footprint (all UNO size)
STACK_H = 42.0            # carrier PCB bottom -> top of Grove plugs + cable bends
CLEAR = 2.0               # gap board <-> wall, each side (v1: 0.8, too tight)
WALL = 2.0
FLOOR = 2.5
STANDOFF_H = 10.0         # carrier underside header/jack are ~8.5 mm tall (v1: 3)
LID_T = 2.5
LIP_DEPTH = 5.0
LIP_T = 1.6
LID_FIT = 0.35            # clearance per side for lid lip (loosen if too tight)
FRONT_BAY = 9.0           # extra length at the front for the OLED behind the front wall

# Grove OLED 0.96" 128x64 (glass 26.7 x 19.3, PCB ~20-23 x ~40). Glass faces the front wall,
# PCB bottom edge sits in a groove, the Grove plug end goes to +Y = your RIGHT when you look at the rover's face
OLED_GAP = 6.0            # wall inner face -> PCB front face (room for the Grove connector)
OLED_WIN = (31.0, 22.0)   # window in the front wall (Y x Z), glass is 26.7 x 19.3
OLED_Z = 27.0             # window centre height

WHEEL_D, WHEEL_W = 30.0, 10.0
PEG_D = 4.0               # axle peg printed on each wheel (points up on the bed)
AXLE_HOLE_D = 4.3         # matching hole through arm + wall; glue the peg in
WHEEL_GAP = 1.0           # spacer collar between wheel and arm
PEG_Z = 6.0               # wheel axle height above body underside
ARM_T = 1.6               # rocker-bogie emboss thickness

CAMERA_MOUNT = True       # under-floor pocket + ribbon slot for a 25x24 cam board
USB_Z = 15.0              # USB-C height above the bottom of the board stack (you measured)
IMU_POCKET = (42.6, 27.0) # IMU cradle under the roof (Modulino Movement, loosened in v2)
IMU_WALL = 3.0
CAM_HOLE_D = 12.0         # lens hole through the floor (fits IMX219 and ESP32-CAM OV2640 lenses)
ARUCO = 40.0              # ArUco marker pad size on the roof

# derived
IN_X = BOARD[0] + 2 * CLEAR + FRONT_BAY
IN_Y = BOARD[1] + 2 * CLEAR
IN_Z = STANDOFF_H + STACK_H + 1.0
OUT_X, OUT_Y, OUT_Z = IN_X + 2 * WALL, IN_Y + 2 * WALL, FLOOR + IN_Z


# ------------------------------------------------------------------ helpers
def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    b = _box(extents=hi - lo)
    b.apply_translation((lo + hi) / 2)
    return b


def cyl(r, h, center, axis="z", sections=64):
    c = _cyl(radius=r, height=h, sections=sections)
    if axis == "x":
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == "y":
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation(center)
    return c


def bar_xz(p1, p2, width, y0, y1):
    """Flat bar in the XZ plane between two points, spanning y0..y1."""
    p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
    d = p2 - p1
    length = np.linalg.norm(d)
    b = _box(extents=[length, y1 - y0, width])
    ang = np.arctan2(d[1], d[0])
    b.apply_transform(trimesh.transformations.rotation_matrix(-ang, [0, 1, 0]))
    mid = (p1 + p2) / 2
    b.apply_translation([mid[0], (y0 + y1) / 2, mid[1]])
    return b


def prism_xz(pts, y0, y1):
    """Extrude a polygon given in the XZ plane along Y from y0 to y1."""
    from shapely.geometry import Polygon
    m = trimesh.creation.extrude_polygon(Polygon(pts), y1 - y0)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))  # poly y->z, extrude z->-y
    m.apply_translation([0, y1, 0])
    return m


def edge_cut(c, axis, at, length):
    """45-degree chamfer cutter of size c along an edge parallel to `axis`, through point `at`."""
    s = c * np.sqrt(2)
    ext = {"x": [length, s, s], "y": [s, length, s], "z": [s, s, length]}[axis]
    b = _box(extents=ext)
    vec = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}[axis]
    b.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 4, vec))
    b.apply_translation(at)
    return b


def union(*m):
    return trimesh.boolean.union([x for x in m], engine="manifold")


def diff(a, *m):
    return trimesh.boolean.difference([a, *m], engine="manifold")


# ------------------------------------------------------------------ body
WHEEL_X = (4.0, OUT_X / 2, OUT_X - 4.0)
OLED_YC = OUT_Y / 2 - 6.0          # screen sits left of centre so the plug end fits on the right
HEAD_Z = OUT_Z - 10.0


def make_body():
    shell = box((0, 0, 0), (OUT_X, OUT_Y, OUT_Z))
    cavity = box((WALL, WALL, FLOOR), (OUT_X - WALL, OUT_Y - WALL, OUT_Z + 1))
    c = 2.4  # chamfer on vertical corners and bottom edges (keeps >=1.4 mm wall)
    chamfers = [edge_cut(c, "z", (x, y, OUT_Z / 2), OUT_Z + 2) for x in (0, OUT_X) for y in (0, OUT_Y)]
    chamfers += [edge_cut(c, "x", (OUT_X / 2, y, 0), OUT_X + 2) for y in (0, OUT_Y)]
    chamfers += [edge_cut(c, "y", (0, OUT_Y / 2, 0), OUT_Y + 2)]
    parts = [diff(shell, cavity, *chamfers)]

    # front bumper wedge with a chamfered nose
    bumper = box((OUT_X - 1, 7, 0), (OUT_X + 5, OUT_Y - 7, 11))
    bumper = diff(bumper, edge_cut(4.5, "y", (OUT_X + 5, OUT_Y / 2, 11), OUT_Y),
                  edge_cut(2.0, "y", (OUT_X + 5, OUT_Y / 2, 0), OUT_Y))
    parts.append(bumper)
    # OLED screen bezel (replaces v1's vent visor)
    oy_c = OLED_YC
    wy, wz = OLED_WIN[0] / 2, OLED_WIN[1] / 2
    bezel = diff(box((OUT_X - 0.5, oy_c - wy - 2.5, OLED_Z - wz - 2.5), (OUT_X + 1.4, oy_c + wy + 2.5, OLED_Z + wz + 2.5)),
                 *[edge_cut(1.2, "z", (OUT_X + 1.4, y, OLED_Z), 40) for y in (oy_c - wy - 2.5, oy_c + wy + 2.5)],
                 *[edge_cut(1.2, "y", (OUT_X + 1.4, oy_c, z), 50) for z in (OLED_Z - wz - 2.5, OLED_Z + wz + 2.5)])
    parts.append(bezel)
    # inside: groove that holds the OLED PCB's bottom edge, 45-degree underside (prints without support)
    xw = OUT_X - WALL
    zg = OLED_Z - 10.8                       # groove floor -> glass centre lands on the window centre
    D = OLED_GAP + 2.2 + 1.2
    zb = zg - 1.5 - D
    shelf = diff(box((xw - D, oy_c - 21.0, zb), (xw + 0.5, oy_c + 21.0, zg + 2.0)),
                 edge_cut(D, "y", (xw - D, oy_c, zb), 50))
    parts.append(diff(shelf, box((xw - OLED_GAP - 2.2, oy_c - 22, zg), (xw - OLED_GAP, oy_c + 22, zg + 5))))
    # headlight bezels (moved up above the screen)
    for y in (OUT_Y * 0.12, OUT_Y * 0.88):
        parts.append(cyl(4.2, 2.4, (OUT_X + 0.6, y, HEAD_Z), axis="x", sections=40))

    # UNO hole pattern standoffs (board origin = USB/power corner at rear-left)
    ox, oy = WALL + CLEAR, WALL + CLEAR
    holes = [(13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56)]
    standoffs = []
    for hx, hy in holes:
        standoffs.append(cyl(3.0, STANDOFF_H + 0.5, (ox + hx, oy + hy, FLOOR + (STANDOFF_H - 0.5) / 2)))
    parts += standoffs

    # rocker-bogie arms embossed on both sides + wheel pegs
    rear, mid, front = [(x, PEG_Z) for x in WHEEL_X]
    pivot = (OUT_X * 0.40, 22.0)
    bogie = ((mid[0] + front[0]) / 2, 15.0)
    links = [(rear, pivot), (pivot, bogie), (bogie, mid), (bogie, front)]
    joints = [rear, mid, front, pivot, bogie]
    for y0, y1, sgn in ((-ARM_T, 0.01, -1), (OUT_Y - 0.01, OUT_Y + ARM_T, 1)):
        for a, b in links:
            parts.append(bar_xz(a, b, 4.5, y0, y1))
        for jx, jz in joints:
            r = 4.5 if (jx, jz) in (pivot, bogie) else 4.0
            parts.append(cyl(r, y1 - y0, (jx, (y0 + y1) / 2, jz), axis="y"))
    body = union(*parts)

    cuts = []
    # trim the side arms flat above the bottom chamfer so nothing starts in mid-air
    cuts += [box((-ARM_T - 1, -ARM_T - 1, -1), (OUT_X + 6, 0, 2.6)),
             box((-ARM_T - 1, OUT_Y, -1), (OUT_X + 6, OUT_Y + ARM_T + 1, 2.6))]
    # axle holes through the arm hubs and side walls (pegs live on the wheels)
    for jx in WHEEL_X:
        for yc in (-ARM_T / 2, OUT_Y + ARM_T / 2):          # only through arm + wall, not the inside
            cuts.append(cyl(AXLE_HOLE_D / 2, ARM_T + 2 * WALL + 2, (jx, yc, PEG_Z), axis="y", sections=32))
    # rear service window: covers the UNO Q USB-C and the Media Carrier's audio jack /
    # connectors on that end at any height you measured, plus the plug overmold
    cuts.append(box((-1, WALL + 5, FLOOR + 3), (WALL + 1, OUT_Y - WALL - 5, FLOOR + STANDOFF_H + 24)))
    # LED strip / extra cable slot, high on the rear wall
    cuts.append(box((-1, OUT_Y / 2 - 6, OUT_Z - 10), (WALL + 1, OUT_Y / 2 + 6, OUT_Z - 5)))
    # OLED window + two 5 mm "headlight" LED holes
    cuts.append(box((OUT_X - WALL - 1, OLED_YC - OLED_WIN[0] / 2, OLED_Z - OLED_WIN[1] / 2),
                    (OUT_X + 3, OLED_YC + OLED_WIN[0] / 2, OLED_Z + OLED_WIN[1] / 2)))
    for y in (OUT_Y * 0.12, OUT_Y * 0.88):
        cuts.append(cyl(2.55, WALL + 6, (OUT_X, y, HEAD_Z), axis="x", sections=32))
    # side cooling slots above the arms
    for x in np.linspace(OUT_X * 0.2, OUT_X * 0.8, 6):
        cuts.append(box((x - 1.2, -ARM_T - 1, OUT_Z - 18), (x + 1.2, OUT_Y + ARM_T + 1, OUT_Z - 8)))
    # standoff pilot holes for M3 self-tapping screws (UNO holes are 3.2 mm)
    for hx, hy in holes:
        cuts.append(cyl(1.3, STANDOFF_H + FLOOR, (ox + hx, oy + hy, FLOOR + STANDOFF_H / 2 + 0.6), sections=24))

    # snap grooves for the lid ridges (inner long walls, just under the rim)
    for x in (OUT_X / 2 - SNAP_DX, OUT_X / 2 + SNAP_DX):
        for y in (WALL, OUT_Y - WALL):
            cuts.append(cyl(1.0, SNAP_LEN + 3, (x, y, OUT_Z - LIP_DEPTH / 2), axis="x", sections=24))

    if CAMERA_MOUNT:
        cx, cy = WALL + CLEAR + 52.0, OUT_Y / 2
        # pocket on the underside for a 25 x 24 camera board (lens faces down)
        cuts.append(box((cx - 12.8, cy - 13.3, -1), (cx + 12.8, cy + 13.3, 1.2)))
        # M2 holes, Pi Cam v2 pattern 12.5 x 21
        for dx in (-6.25, 6.25):
            for dy in (-10.5, 10.5):
                cuts.append(cyl(1.1, FLOOR + 2, (cx + dx, cy + dy, FLOOR / 2), sections=20))
        # lens hole straight through the floor (camera looks down)
        cuts.append(cyl(CAM_HOLE_D / 2, FLOOR + 4, (cx, cy, FLOOR / 2), sections=48))
        # ribbon cable slot into the box
        cuts.append(box((cx - 15.5, cy - 9, -1), (cx - 12.5, cy + 9, FLOOR + 1)))
    return diff(body, *cuts)


# ------------------------------------------------------------------ lid
SOCKET = 6.4                     # square socket for all roof accessories (pegs are 6.0)
MAST_X = OUT_X - 12.0
MAST_POS = (MAST_X, OUT_Y / 2)
RTG_POS = (7.5, 8.0)
DISH_POS = (7.5, OUT_Y - 8.0)
WING = 11.0
SNAP_R, SNAP_LEN, SNAP_DX = 0.7, 10.0, 18.0     # lid-to-body snap ridges (v2.1)


def peg():
    return box((-3.0, -3.0, -LID_T - 2.0), (3.0, 3.0, 0.01))


def make_lid():
    plate = box((0, -WING, 0), (OUT_X, OUT_Y + WING, LID_T))
    cuts = [box((-1, -WING - 1, -1), (OUT_X + 1, -0.5, 0.8)),            # thin solar wings
            box((-1, OUT_Y + 0.5, -1), (OUT_X + 1, OUT_Y + WING + 1, 0.8))]
    # angled wing tips + chamfered roof edge (roof edge is on the bed when printing)
    for x in (0, OUT_X):
        for y in (-WING, OUT_Y + WING):
            cuts.append(edge_cut(7.0, "z", (x, y, LID_T / 2), LID_T + 2))
        cuts.append(edge_cut(1.0, "y", (x, OUT_Y / 2, LID_T), OUT_Y + 2 * WING + 2))
    plate = diff(plate, *cuts)

    lo = np.array([WALL + LID_FIT, WALL + LID_FIT])
    hi = np.array([OUT_X - WALL - LID_FIT, OUT_Y - WALL - LID_FIT])
    lip = diff(box((*lo, -LIP_DEPTH), (*hi, 0.01)),
               box((*(lo + LIP_T), -LIP_DEPTH - 1), (*(hi - LIP_T), 1)))
    # IMU cradle under the roof (Modulino Movement ~41 x 25.5 mm): stick the board in with
    # double-sided tape; notch in one end for the Qwiic cable
    cx, cy = 34.0, OUT_Y / 2
    iw, ih, t = IMU_POCKET[0] / 2, IMU_POCKET[1] / 2, 1.2
    cradle = diff(box((cx - iw - t, cy - ih - t, -IMU_WALL), (cx + iw + t, cy + ih + t, 0.01)),
                  box((cx - iw, cy - ih, -IMU_WALL - 1), (cx + iw, cy + ih, 0.5)),
                  box((cx + iw - 1, cy - 4, -IMU_WALL - 1), (cx + iw + t + 1, cy + 4, 0.5)))
    # snap ridges on the long sides of the lip -> click into grooves in the body, so the
    # rover can hang from the lid (sky crane). ~0.35 mm interference.
    ridges = [cyl(SNAP_R, SNAP_LEN, (x, y, -LIP_DEPTH / 2), axis="x", sections=24)
              for x in (OUT_X / 2 - SNAP_DX, OUT_X / 2 + SNAP_DX) for y in (lo[1], hi[1])]
    lid = union(plate, lip, cradle, *ridges)

    cuts = []
    # ArUco marker pad (0.6 mm deep) with a thin engraved frame around it
    ax0, ay0 = 14.0, OUT_Y / 2 - ARUCO / 2
    cuts.append(box((ax0, ay0, LID_T - 0.6), (ax0 + ARUCO, ay0 + ARUCO, LID_T + 1)))
    f = 1.8
    cuts.append(diff(box((ax0 - f - 0.6, ay0 - f - 0.6, LID_T - 0.4), (ax0 + ARUCO + f + 0.6, ay0 + ARUCO + f + 0.6, LID_T + 1)),
                     box((ax0 - f, ay0 - f, LID_T - 1), (ax0 + ARUCO + f, ay0 + ARUCO + f, LID_T + 2))))
    # solar cell grid on both wings
    for y_lo, y_hi in ((-WING + 1.2, -1.5), (OUT_Y + 1.5, OUT_Y + WING - 1.2)):
        for x in np.arange(5.5, OUT_X - 3, 6.2):
            cuts.append(box((x, y_lo, LID_T - 0.5), (x + 0.6, y_hi, LID_T + 1)))
        ym = (y_lo + y_hi) / 2
        cuts.append(box((1, ym - 0.3, LID_T - 0.5), (OUT_X - 1, ym + 0.3, LID_T + 1)))
    # accessory sockets + finger notch to pry the lid off
    for px, py in (MAST_POS, RTG_POS, DISH_POS):
        cuts.append(box((px - SOCKET / 2, py - SOCKET / 2, -1), (px + SOCKET / 2, py + SOCKET / 2, LID_T + 1)))
    cuts.append(box((-1, OUT_Y / 2 - 6, -1), (1.5, OUT_Y / 2 + 6, 1.2)))
    return diff(lid, *cuts)


# ------------------------------------------------------------------ roof accessories (z=0 = roof surface)
def make_mast_post():
    """White mast post: roof peg, collar, neck and pan block with a socket for the red head."""
    collar = cyl(5.0, 3.0, (0, 0, 1.5), sections=48)
    neck = cyl(3.8, 18.5, (0, 0, 9.25 + 0.0), sections=48)
    pan = box((-4.5, -5, 18), (4.5, 5, 24))                            # pan/tilt block
    post = union(peg(), collar, neck, pan)
    return diff(post, box((-3.1, -3.1, 19.8), (3.1, 3.1, 25)))        # 6.2 mm head socket


def make_mast_head():
    """Red camera head (stereo cams + sensor pod) with a tenon that plugs into the post."""
    head = box((-7, -11, 24), (8, 11, 35))
    head = diff(head, *[edge_cut(2.0, "z", (x, y, 29.5), 13) for x in (-7, 8) for y in (-11, 11)],
                edge_cut(2.5, "y", (8, 0, 35), 24))
    side_box = box((-5, 11 - 0.01, 26), (3, 14, 33))                  # sensor pod
    tenon = box((-2.9, -2.9, 20.2), (2.9, 2.9, 24.01))                # 5.8 into 6.2 socket
    m = union(head, side_box, tenon)
    cuts = [cyl(2.6, 6, (8, y, 29.5), axis="x", sections=40) for y in (-5.2, 5.2)]   # stereo cams
    cuts += [cyl(3.6, 1.2, (8.2, y, 29.5), axis="x", sections=40) for y in (-5.2, 5.2)]  # lens bezels
    cuts.append(box((-7.5, -8, 33), (-5.5, 8, 33.8)))                 # vent line on the back
    return diff(m, *cuts)


def make_dish():
    stalk = cyl(2.8, 16.0, (0, 0, 8.0), sections=40)   # v2: thicker (v1 broke)
    outer = trimesh.creation.cone(radius=12.0, height=12.0, sections=96)
    outer.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    outer.apply_translation([0, 0, 26.0])                             # apex at z=14, rim at z=26
    inner = trimesh.creation.cone(radius=10.9, height=7.0, sections=96)
    inner.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    inner.apply_translation([0, 0, 26.01])
    dish = diff(union(peg(), stalk, outer), inner)
    feed = union(cyl(1.3, 9.0, (0, 0, 23.5), sections=24), cyl(1.8, 1.6, (0, 0, 28.0), sections=32))
    return union(dish, feed)


def make_rtg():
    base = cyl(5.5, 2.0, (0, 0, 1.0), sections=48)
    core = cyl(4.0, 18.0, (0, 0, 11.0), sections=48)
    fins = []
    for i in range(8):
        f = box((3.0, -0.7, 3.5), (7.5, 0.7, 18.5))   # v2: thicker fins
        f.apply_transform(trimesh.transformations.rotation_matrix(2 * np.pi * i / 8 + np.pi / 8, [0, 0, 1]))
        fins.append(f)
    cap = cyl(5.0, 2.2, (0, 0, 21.1), sections=48)
    nub = cyl(2.0, 1.5, (0, 0, 22.9), sections=32)
    return union(peg(), base, core, *fins, cap, nub)


# ------------------------------------------------------------------ wheel
def make_wheel():
    r = WHEEL_D / 2
    rim_in = r - 2.8
    rim = diff(cyl(r, WHEEL_W, (0, 0, WHEEL_W / 2), sections=120),
               cyl(rim_in, WHEEL_W + 2, (0, 0, WHEEL_W / 2), sections=120))
    # chevron tread: two staggered rows of notches
    cuts, n = [], 14
    for row, (z0, z1) in enumerate(((-1, WHEEL_W / 2 + 0.6), (WHEEL_W / 2 - 0.6, WHEEL_W + 1))):
        for i in range(n):
            a = 2 * np.pi * (i + 0.5 * row) / n
            g = box((r - 1.0, -1.1, z0), (r + 1.0, 1.1, z1))
            g.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1]))
            cuts.append(g)
    rim = diff(rim, *cuts)
    # curved full-width spokes (no web, so it prints without bridges)
    spokes = []
    for k in range(6):
        a0 = 2 * np.pi * k / 6
        pts = [(4.0 + 8.6 * t, a0 + 0.7 * t) for t in np.linspace(0, 1, 5)]
        for (r1, a1), (r2, a2) in zip(pts, pts[1:]):
            p1 = np.array([r1 * np.cos(a1), r1 * np.sin(a1)])
            p2 = np.array([r2 * np.cos(a2), r2 * np.sin(a2)])
            L = np.linalg.norm(p2 - p1) + 1.6
            s = _box(extents=[L, 2.0, WHEEL_W])
            s.apply_transform(trimesh.transformations.rotation_matrix(np.arctan2(*(p2 - p1)[::-1]), [0, 0, 1]))
            s.apply_translation([*(p1 + p2) / 2, WHEEL_W / 2])
            spokes.append(s)
    hub = cyl(5.0, WHEEL_W, (0, 0, WHEEL_W / 2), sections=48)
    # inboard side (z < 0): spacer collar + axle peg that goes through arm and wall
    collar = cyl(3.2, WHEEL_GAP, (0, 0, -WHEEL_GAP / 2), sections=48)
    peg_len = ARM_T + WALL - 0.2
    axle = cyl(PEG_D / 2, peg_len, (0, 0, -WHEEL_GAP - peg_len / 2), sections=32)
    w = union(rim, hub, collar, axle, *spokes)
    # decorative hub recess on the outboard face (z = WHEEL_W), which goes on the bed
    return diff(w, cyl(2.5, 1.2, (0, 0, WHEEL_W - 0.4), sections=32))


# ------------------------------------------------------------------ sky crane (descent stage)
# Coordinates: same x/y as the lid, z = 0 at the roof surface. The four bridle legs end in
# 45-degree ramps that catch the outer edges of the lid's solar wings (z -1.7..0) and centre
# the rover. After touchdown: lower the crane ~3 mm and slide it forward (+x) = "fly away".
CLIP_GAP = 0.3          # lip clearance above the wing top (slot 2.0 mm for the 1.7 mm wing)
DETENT_H = 0.6          # click bump drop below the lip -> 0.3 mm interference with the wing
THRUSTER_CANT = 25.0    # degrees the nozzles tilt outward
POD_OUT = 11.0          # how far the pods sit out from the deck corners
SIMPLE_CRANE_Z = 39.0   # simple crane deck underside (mast head top is at 35)
CRANE_Z = 50.0          # deck underside above the roof (mast head top is at 35)
CRANE_T = 3.0
CLIP_X = ((10.0, 24.0), (OUT_X - 20.0, OUT_X - 3.7))   # rear clips, front clips (front ones have a stop)


def make_skycrane_detailed():
    x0, x1 = -8.0, OUT_X + 8.0
    y0, y1 = -WING - 9.0, OUT_Y + WING + 9.0
    deck = box((x0, y0, CRANE_Z), (x1, y1, CRANE_Z + CRANE_T))
    deck = diff(deck, *[edge_cut(8.0, "z", (x, y, CRANE_Z + CRANE_T / 2), 10) for x in (x0, x1) for y in (y0, y1)])
    # truss look: four lightening windows around a solid centre hub (stick an ArUco tag on top)
    xm, ym, hub, fr = OUT_X / 2, OUT_Y / 2, 26.0, 7.0
    wins = []
    for (a, b) in ((x0 + fr, xm - hub), (xm + hub, x1 - fr)):
        for (c, d) in ((y0 + fr, ym - hub), (ym + hub, y1 - fr)):
            w = box((a, c, CRANE_Z - 1), (b, d, CRANE_Z + CRANE_T + 1))
            wins.append(diff(w, *[edge_cut(4.0, "z", (x, y, CRANE_Z + 1), 10) for x in (a, b) for y in (c, d)]))
    deck = diff(deck, *wins)
    parts = [deck]

    def leg_and_clip(xa, xb, side, stop):
        s = -1 if side < 0 else 1
        Y = (lambda y: y) if side < 0 else (lambda y: OUT_Y - y)          # mirror for the +y side
        def yb(a, b):
            ya, yb_ = sorted((Y(a), Y(b)))
            return ya, yb_
        out = []
        ly = yb(-WING - 6.5, -WING - 3.0)
        out.append(box((xa, ly[0], -6.7), (xb, ly[1], CRANE_Z + 0.1)))                 # bridle leg
        wy = yb(-WING - 3.0, -WING + 3.0)
        ramp = box((xa, wy[0], -6.7), (xb, wy[1], 1.3))
        corner_y = Y(-WING + 3.0)
        out.append(diff(ramp, edge_cut(6.0, "x", ((xa + xb) / 2, corner_y, 1.3), xb - xa + 2)))
        if stop:
            sy = yb(-WING - 6.5, -WING + 3.0)
            out.append(box((xb, sy[0], -6.7), (xb + 2.0, sy[1], 2.0)))
        return out

    for (xa, xb), stop in zip(CLIP_X, (False, True)):
        for side in (-1, 1):
            parts += leg_and_clip(xa, xb, side, stop)
    # thruster pods: paired nozzles at each corner, pointing down
    for px in (x0 + 7, x1 - 7):
        for py in (y0 + 7, y1 - 7):
            parts.append(box((px - 7, py - 4.5, CRANE_Z - 7), (px + 7, py + 4.5, CRANE_Z + CRANE_T)))
            for dx in (-3.6, 3.6):
                parts.append(cyl(2.4, 6.0, (px + dx, py, CRANE_Z - 10.0), sections=32))   # throat
                cone = trimesh.creation.cone(radius=3.4, height=5.0, sections=40)
                cone.apply_translation([px + dx, py, CRANE_Z - 15.0])        # solid bell, flares downward
                parts.append(diff(cone, cyl(2.4, 2.0, (px + dx, py, CRANE_Z - 15.0), sections=32)))
    # propellant tanks under the hub
    for tx in (xm - 16, xm + 16):
        tank = trimesh.creation.icosphere(subdivisions=4, radius=9.0)
        tank.apply_translation([tx, ym, CRANE_Z + 0.5])
        parts.append(diff(tank, box((tx - 10, ym - 10, CRANE_Z + 0.4), (tx + 10, ym + 10, CRANE_Z + 20))))
    return union(*parts)


def nozzle_bell(r_throat=1.7, r_exit=3.3, length=6.5, wall=0.8, sections=48):
    """Rocket nozzle along -z from z=0: throat collar + flared bell, hollow, open at the exit."""
    ze = -1.6 - length                                                   # exit plane
    collar = cyl(r_throat + 0.9, 1.6, (0, 0, -0.8), sections=sections)
    outer = trimesh.creation.cone(radius=r_exit, height=length * r_exit / (r_exit - r_throat), sections=sections)
    outer.apply_translation([0, 0, ze])                                  # wide end down, apex up
    outer = diff(outer, box((-9, -9, -1.6), (9, 9, 30)))                 # trim at the throat
    lip = diff(cyl(r_exit + 0.35, 0.8, (0, 0, ze + 0.4), sections=sections),
               cyl(r_exit - wall, 2, (0, 0, ze + 0.4), sections=sections))
    r_in = r_exit - wall
    inner = trimesh.creation.cone(radius=r_in, height=length * r_in / (r_in - 1.1), sections=sections)
    inner.apply_translation([0, 0, ze - 0.01])
    inner = diff(inner, box((-9, -9, -1.6), (9, 9, 30)))
    return diff(union(collar, outer), inner)


def thruster_pod(corner, outward, zc, t):
    """Descent-stage thruster pod on a diagonal outrigger arm: octagonal engine housing with two
    separate nozzles side by side, both tilted outward. Prints deck-down, no supports."""
    o = np.array([*outward, 0.0]) / np.linalg.norm(outward)          # unit, pointing away from the crane
    pdir = np.array([-o[1], o[0], 0.0])                              # across the arm
    c = np.array([*corner, 0.0]) + o * POD_OUT                       # pod centre (x, y)
    out = []
    arm_len = POD_OUT + 6.0
    arm = _box(extents=[arm_len, 10.0, t])
    arm.apply_transform(trimesh.transformations.rotation_matrix(np.arctan2(o[1], o[0]), [0, 0, 1]))
    mid = np.array([*corner, 0.0]) + o * (POD_OUT - 6.0) / 2 - o * 3.0 + o * 3.0
    arm.apply_translation([mid[0], mid[1], zc + t / 2])
    out.append(arm)
    rot = trimesh.transformations.rotation_matrix(np.arctan2(o[1], o[0]) + np.pi / 8, [0, 0, 1])
    for r, z0, z1 in ((7.9, zc - 3.5, zc + t), (7.3, zc - 5.0, zc - 3.4)):    # stepped octagon housing
        h = cyl(r, z1 - z0, (0, 0, (z0 + z1) / 2), sections=8)
        h.apply_transform(rot); h.apply_translation([c[0], c[1], 0])
        out.append(h)
    tilt = np.radians(THRUSTER_CANT)
    axis = np.cross([0, 0, 1.0], o)                                  # rotating -z about this tips the exhaust outward
    for side in (-1, 1):
        n = nozzle_bell(r_throat=1.6, r_exit=3.0, length=6.0)
        n.apply_transform(trimesh.transformations.rotation_matrix(-tilt, axis))
        pos = c + pdir * side * 3.9                                 # 1.8 mm gap between the bells
        n.apply_translation([pos[0], pos[1], zc - 5.0 + 1.6])         # collar sunk 1.6 mm into the housing
        out.append(n)
    return out


def crane_clip(xa, xb, side, stop, detent, zc):
    """C-slot around the wing edge: 45-degree ramp below (centres the rover when lifted),
    lip above (crane rests on the wings hands-free), front stop or rear click bump."""
    Y = (lambda y: y) if side < 0 else (lambda y: OUT_Y - y)
    yb = lambda a, b: tuple(sorted((Y(a), Y(b))))
    out = []
    ly = yb(-WING - 6.0, -WING - 3.0)
    out.append(box((xa, ly[0], -6.7), (xb, ly[1], zc + 0.1)))                 # bridle leg 3 mm thick
    wy = yb(-WING - 3.0, -WING + 3.0)
    out.append(diff(box((xa, wy[0], -6.7), (xb, wy[1], 1.3)),                 # ramp under the wing edge
                    edge_cut(6.0, "x", ((xa + xb) / 2, Y(-WING + 3.0), 1.3), xb - xa + 2)))
    lz = CLIP_GAP                                                             # lip underside, wing top is z = 0
    lpy = yb(-WING - 3.0, -WING + 2.0)
    lip = diff(box((xa, lpy[0], lz), (xb, lpy[1], lz + 7.0)),                 # lip + 45-degree gusset
               edge_cut(5.0, "x", ((xa + xb) / 2, Y(-WING + 2.0), lz + 7.0), xb - xa + 2))
    out.append(lip)
    if stop:
        sy = yb(-WING - 6.0, -WING + 3.0)
        out.append(box((xb, sy[0], -6.7), (xb + 2.0, sy[1], lz + 7.0)))
    if detent:                                                                # click bump just behind the wing tip
        hy = yb(-WING - 3.0, -WING + 1.0)
        r = 1.5
        out.append(cyl(r, hy[1] - hy[0], (xa + 2.0, (hy[0] + hy[1]) / 2, lz + r - DETENT_H), axis="y", sections=32))
    return out


def make_skycrane():
    """Simple, fast sky crane (~1 h print): open frame deck + hub for an ArUco tag, 4 bridle legs
    with the same 45-degree wing clips, 4 nozzle stubs. Prints deck-down, no supports."""
    zc, t = SIMPLE_CRANE_Z, 1.6
    x0, x1 = -4.0, OUT_X + 4.0
    y0, y1 = -WING - 7.5, OUT_Y + WING + 7.5
    xm, ym, rw = OUT_X / 2, OUT_Y / 2, 5.0
    ring = diff(box((x0, y0, zc), (x1, y1, zc + t)), box((x0 + rw, y0 + rw, zc - 1), (x1 - rw, y1 - rw, zc + t + 1)),
                *[edge_cut(3.0, "z", (x, y, zc + t / 2), 10) for x in (x0, x1) for y in (y0, y1)])
    parts = [ring,
             box((x0, ym - 3, zc), (x1, ym + 3, zc + t)),               # cross bars
             box((xm - 3, y0, zc), (xm + 3, y1, zc + t)),
             box((xm - 21, ym - 21, zc), (xm + 21, ym + 21, zc + t))]   # hub: stick a 40 mm ArUco on top

    for (xa, xb), stop in zip(((2.5, 15.0), (OUT_X - 15.7, OUT_X - 3.7)), (False, True)):
        for side in (-1, 1):
            parts += crane_clip(xa, xb, side, stop, detent=not stop, zc=zc)
    for cx_ in (x0 + 2.0, x1 - 2.0):                                           # thruster pods on corner outriggers
        for cy_ in (y0 + 2.0, y1 - 2.0):
            parts += thruster_pod((cx_, cy_), (np.sign(cx_ - xm), np.sign(cy_ - ym)), zc, t)
    return union(*parts)


def make_clip_test():
    """~10 min fit test: a lid-wing section (with the real rear wing-tip chamfer) and one rear clip.
    Slide the wing in from the clip's open end until it clicks past the bump."""
    wing = box((0, -WING, 0.8), (26, 6, LID_T))                         # wing 1.7 thick, as on the lid
    wing = union(wing, box((0, -0.5, 0), (26, 6, LID_T)))                # strip of roof plate
    wing = diff(wing, edge_cut(7.0, "z", (0, -WING, LID_T / 2), LID_T + 2))
    wing.apply_translation([0, 0, -LID_T])                               # roof surface at z = 0
    zc = 8.0
    clip = union(*crane_clip(2.5, 15.0, -1, False, True, zc),
                 box((1.0, -WING - 7.5, zc), (16.5, -WING + 3.5, zc + 2.0)))   # small base plate (bed side)
    return wing, clip


# ------------------------------------------------------------------ assembly (preview only)
def assembly(parts_by_name):
    p = parts_by_name
    out = [(p["body"], "body")]
    lid_a = p["lid"].copy(); lid_a.apply_translation([0, 0, OUT_Z]); out.append((lid_a, "lid"))
    for name, (x, y) in (("mast_post", MAST_POS), ("mast_head", MAST_POS), ("rtg", RTG_POS), ("dish", DISH_POS)):
        a = p[name].copy(); a.apply_translation([x, y, OUT_Z + LID_T]); out.append((a, name))
    for x in WHEEL_X:
        for side in (-1, 1):
            wa = p["wheel"].copy()
            wa.apply_transform(trimesh.transformations.rotation_matrix(-side * np.pi / 2, [1, 0, 0]))
            y = -ARM_T - WHEEL_GAP if side < 0 else OUT_Y + ARM_T + WHEEL_GAP
            wa.apply_translation([x, y, PEG_Z])
            out.append((wa, "wheel"))
    return out


def flip(m):
    m = m.copy()
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    return m


if __name__ == "__main__":
    out = Path(__file__).parent / "stl"
    out.mkdir(exist_ok=True)
    parts = {"body": make_body(), "lid": make_lid(), "mast_post": make_mast_post(), "mast_head": make_mast_head(),
             "dish": make_dish(), "rtg": make_rtg(), "wheel": make_wheel(), "skycrane": make_skycrane()}
    _tw, _tc = make_clip_test()
    parts.update(test_wing=_tw, test_clip=_tc)
    # print orientations: lid roof-down, mast head-down, others as modelled
    printable = dict(parts, lid=flip(parts["lid"]), mast_post=flip(parts["mast_post"]), mast_head=flip(parts["mast_head"]), wheel=flip(parts["wheel"]), skycrane=flip(parts["skycrane"]), test_wing=flip(parts["test_wing"]), test_clip=flip(parts["test_clip"]))
    for name, m in printable.items():
        m = m.copy()
        m.apply_translation([0, 0, -m.bounds[0][2]])
        m.export(out / f"{name}.stl")
        print(f"{name:6s} watertight={m.is_watertight}  size={np.round(m.extents, 1)} mm")
    print(f"inner cavity {IN_X:.1f} x {IN_Y:.1f} x {IN_Z:.1f} mm, outer body {OUT_X:.1f} x {OUT_Y:.1f} x {OUT_Z:.1f} mm")
    asm_dir = out.parent / "assembly"
    asm_dir.mkdir(exist_ok=True)
    desc_dir = out.parent / "assembly_descent"          # rover hanging under the sky crane
    desc_dir.mkdir(exist_ok=True)
    rover_asm = assembly({k: v for k, v in parts.items() if k not in ("skycrane", "test_wing", "test_clip")})
    crane_a = parts["skycrane"].copy(); crane_a.apply_translation([0, 0, OUT_Z + LID_T])
    for i, (m, name) in enumerate(rover_asm + [(crane_a, "skycrane")]):
        m.export(desc_dir / f"{i:02d}_{name}.stl")
    for i, (m, name) in enumerate(assembly({k: v for k, v in parts.items() if k not in ("skycrane", "test_wing", "test_clip")})):
        m.export(asm_dir / f"{i:02d}_{name}.stl")


# ------------------------------------------------------------------ one-plate layout (Bambu A1: 256 x 256 mm)
BED = 256.0


def build_plate(stl_dir):
    """Arrange every printable part (6 wheels) on one A1 plate. Returns list of (name, mesh)."""
    load = lambda n: trimesh.load(stl_dir / f"{n}.stl")
    gap = 6.0
    rows = [["body", "lid"], ["wheel"] * 6, ["mast_post", "mast_head", "dish", "rtg"]]
    placed, y = [], 0.0
    for row in rows:
        x, row_h = 0.0, 0.0
        for i, name in enumerate(row):
            m = load(name)
            lo, hi = m.bounds
            m.apply_translation([x - lo[0], y - lo[1], -lo[2]])
            placed.append((f"{name}{i + 1}" if name == "wheel" else name, m))
            x += (hi - lo)[0] + gap
            row_h = max(row_h, (hi - lo)[1])
        y += row_h + gap
    allb = np.vstack([m.bounds for _, m in placed])
    span = allb.max(0) - allb.min(0)
    assert span[0] <= BED - 20 and span[1] <= BED - 20, f"layout {span} too big for A1"
    shift = np.array([BED / 2, BED / 2, 0]) - np.array([*(allb.min(0) + span / 2)[:2], 0])
    for _, m in placed:
        m.apply_translation(shift)
    return placed, span


if __name__ == "__main__":
    placed, span = build_plate(out)
    scene = trimesh.Scene()
    for name, m in placed:
        scene.add_geometry(m, node_name=name, geom_name=name)
    scene.export(out.parent / "rover_A1_one_plate.3mf")
    trimesh.util.concatenate([m for _, m in placed]).export(out.parent / "rover_A1_one_plate.stl")
    print(f"one-plate layout {span[0]:.0f} x {span[1]:.0f} mm on a {BED:.0f} mm bed, {len(placed)} objects")


# ------------------------------------------------------------------ multi-color plate (AMS lite)
FILAMENTS = [("white", "#FFFFFF"), ("red", "#C12E1F"), ("black", "#161616")]   # AMS slot 1, 2, 3
COLOR_OF = {"body": 1, "dish": 1, "rtg": 1, "mast_post": 1, "lid": 2, "mast_head": 2, "wheel": 3, "skycrane": 3, "test_wing": 3, "test_clip": 3}


def write_bambu_3mf(placed, path):
    """Minimal Bambu-Studio-compatible 3MF: one object per part, each assigned an AMS filament."""
    import zipfile
    objs, items, cfg = [], [], []
    for i, (name, m) in enumerate(placed, start=1):
        v = "".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in m.vertices)
        t = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces)
        objs.append(f'<object id="{i}" name="{name}" type="model"><mesh><vertices>{v}</vertices>'
                    f'<triangles>{t}</triangles></mesh></object>')
        items.append(f'<item objectid="{i}" printable="1"/>')
        ext = COLOR_OF[name.rstrip("0123456789")]
        cfg.append(f'<object id="{i}"><metadata key="name" value="{name}"/><metadata key="extruder" value="{ext}"/>'
                   f'<part id="{i}" subtype="normal_part"><metadata key="name" value="{name}"/>'
                   f'<metadata key="extruder" value="{ext}"/></part></object>')
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
    plate = "".join(f'<model_instance><metadata key="object_id" value="{i}"/><metadata key="instance_id" value="0"/>'
                    f'</model_instance>' for i in range(1, len(placed) + 1))
    settings = (f'<?xml version="1.0" encoding="UTF-8"?><config>{"".join(cfg)}'
                f'<plate><metadata key="plater_id" value="1"/>{plate}</plate></config>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                   '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                   '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                   '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model", model)
        z.writestr("Metadata/model_settings.config", settings)


if __name__ == "__main__":
    write_bambu_3mf(placed, out.parent / "rover_A1_color_plate.3mf")
    allb = np.vstack([m.bounds for _, m in placed])
    print("color plate written; parts occupy x %.0f-%.0f, y %.0f-%.0f" % (allb[0, 0], allb[:, 0].max(), allb[0, 1] if False else allb[:, 1].min(), allb[:, 1].max()))


# ------------------------------------------------------------------ one file, one plate per color (no AMS swaps)
PLATE_STRIDE = BED * 1.2          # Bambu Studio lays plates out on a grid with a 1/5 gap
COLOR_PLATES = [["body", "dish", "rtg", "mast_post"], ["lid", "mast_head"], ["wheel"] * 6]


def build_color_plates(stl_dir):
    load = lambda n: trimesh.load(stl_dir / f"{n}.stl")
    cols = int(np.ceil(np.sqrt(len(COLOR_PLATES))))
    placed, plates = [], []
    for p, names in enumerate(COLOR_PLATES):
        meshes, x, y, row_h, gap = [], 0.0, 0.0, 0.0, 8.0
        for i, n in enumerate(names):
            m = load(n)
            lo, hi = m.bounds
            if x + (hi - lo)[0] > 200:          # wrap rows (wheels)
                x, y, row_h = 0.0, y + row_h + gap, 0.0
            m.apply_translation([x - lo[0], y - lo[1], -lo[2]])
            meshes.append((f"{n}{i + 1}" if n == "wheel" else n, m))
            x += (hi - lo)[0] + gap
            row_h = max(row_h, (hi - lo)[1])
        allb = np.vstack([m.bounds for _, m in meshes])
        c = (allb.min(0) + allb.max(0)) / 2
        col, row = p % cols, p // cols
        off = np.array([BED / 2 - c[0] + col * PLATE_STRIDE, BED / 2 - c[1] - row * PLATE_STRIDE, 0])
        ids = []
        for name, m in meshes:
            m.apply_translation(off)
            placed.append((name, m))
            ids.append(len(placed))
        plates.append(ids)
    return placed, plates


def write_bambu_3mf_plates(placed, plates, path):
    import zipfile, io
    write_bambu_3mf(placed, path)                       # objects + colors, then rewrite plate list
    with zipfile.ZipFile(path) as z:
        files = {n: z.read(n) for n in z.namelist()}
    cfg = files["Metadata/model_settings.config"].decode()
    cfg = cfg[: cfg.index("<plate>")]
    for p, ids in enumerate(plates, start=1):
        inst = "".join(f'<model_instance><metadata key="object_id" value="{i}"/><metadata key="instance_id" value="0"/>'
                       f'</model_instance>' for i in ids)
        cfg += f'<plate><metadata key="plater_id" value="{p}"/><metadata key="plater_name" value="{["white", "red", "black"][p - 1]}"/>{inst}</plate>'
    files["Metadata/model_settings.config"] = (cfg + "</config>").encode()
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for n, d in files.items():
            z.writestr(n, d)


if __name__ == "__main__":
    cp, plates = build_color_plates(out)
    write_bambu_3mf_plates(cp, plates, out.parent / "rover_A1_3plates.3mf")
    print("3-plate file written:", [len(p) for p in plates], "objects per plate")
