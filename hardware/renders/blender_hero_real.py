"""Photoreal-ish hero render of the LUNA-GAP rover on the Moon (Blender 5.x, Cycles).

blender -b -P blender_hero_real.py -- <assembly_dir> <out.png> [W H SAMPLES]
"""
import bpy, bmesh, glob, os, sys, math, random
from mathutils import Vector, noise

argv = sys.argv[sys.argv.index("--") + 1:]
asm_dir, out_png = argv[0], argv[1]
W, H = (int(argv[2]), int(argv[3])) if len(argv) > 3 else (2560, 1440)
SAMPLES = int(argv[4]) if len(argv) > 4 else 256
RES = int(argv[5]) if len(argv) > 5 else 900          # terrain grid resolution
LIFT = float(argv[6]) if len(argv) > 6 else 0.0       # >0: rover hangs under the sky crane, this high (scene units)
random.seed(11)
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
GROUND_Z = -0.09                                       # wheel bottoms (model units x 0.01)


def nodes_mat(name):
    m = bpy.data.materials.new(name); m.use_nodes = True
    return m, m.node_tree.nodes, m.node_tree.links, m.node_tree.nodes["Principled BSDF"]


# ------------------------------------------------------------------ rover materials
def plastic(name, col, rough, dust_amt, layer_bump=0.12):
    m, n, l, b = nodes_mat(name)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Specular IOR Level"].default_value = 0.45
    tc = n.new("ShaderNodeTexCoord")
    # FDM layer lines: object space is in mm, 0.2 mm layers along Z
    sep = n.new("ShaderNodeSeparateXYZ"); l.new(tc.outputs["Object"], sep.inputs[0])
    mul = n.new("ShaderNodeMath"); mul.operation = "MULTIPLY"; mul.inputs[1].default_value = 2 * math.pi / 0.2
    l.new(sep.outputs["Z"], mul.inputs[0])
    sn = n.new("ShaderNodeMath"); sn.operation = "SINE"; l.new(mul.outputs[0], sn.inputs[0])
    bump = n.new("ShaderNodeBump"); bump.inputs["Strength"].default_value = layer_bump; bump.inputs["Distance"].default_value = 0.0004
    l.new(sn.outputs[0], bump.inputs["Height"])
    # rounded printed edges
    bev = n.new("ShaderNodeBevel"); bev.inputs["Radius"].default_value = 0.005; bev.samples = 8
    l.new(bev.outputs["Normal"], bump.inputs["Normal"])
    l.new(bump.outputs["Normal"], b.inputs["Normal"])
    # regolith dust: heavier low down, blotchy
    geo = n.new("ShaderNodeNewGeometry")
    sepw = n.new("ShaderNodeSeparateXYZ"); l.new(geo.outputs["Position"], sepw.inputs[0])
    hmap = n.new("ShaderNodeMapRange"); hmap.inputs["From Min"].default_value = GROUND_Z
    hmap.inputs["From Max"].default_value = GROUND_Z + 0.35; hmap.inputs["To Min"].default_value = 1.0
    hmap.inputs["To Max"].default_value = 0.0
    l.new(sepw.outputs["Z"], hmap.inputs["Value"])
    nz = n.new("ShaderNodeTexNoise"); nz.inputs["Scale"].default_value = 60; nz.inputs["Detail"].default_value = 8
    l.new(geo.outputs["Position"], nz.inputs["Vector"])
    nz2 = n.new("ShaderNodeMath"); nz2.operation = "MULTIPLY_ADD"
    l.new(nz.outputs["Fac"], nz2.inputs[0]); nz2.inputs[1].default_value = 0.55; nz2.inputs[2].default_value = -0.12
    dust = n.new("ShaderNodeMath"); dust.operation = "MULTIPLY"; dust.use_clamp = True
    l.new(hmap.outputs["Result"], dust.inputs[0]); l.new(nz2.outputs[0], dust.inputs[1])
    dmul = n.new("ShaderNodeMath"); dmul.operation = "MULTIPLY"; dmul.use_clamp = True; dmul.inputs[1].default_value = dust_amt
    l.new(dust.outputs[0], dmul.inputs[0])
    mix = n.new("ShaderNodeMix"); mix.data_type = "RGBA"
    mix.inputs["A"].default_value = col; mix.inputs["B"].default_value = (0.36, 0.34, 0.32, 1)
    l.new(dmul.outputs[0], mix.inputs["Factor"]); l.new(mix.outputs["Result"], b.inputs["Base Color"])
    rmix = n.new("ShaderNodeMix"); rmix.inputs["A"].default_value = rough; rmix.inputs["B"].default_value = 0.95
    l.new(dmul.outputs[0], rmix.inputs["Factor"]); l.new(rmix.outputs["Result"], b.inputs["Roughness"])
    return m


white = plastic("pla_white", (0.80, 0.80, 0.77, 1), 0.36, 1.2, 0.25)
red = plastic("pla_red", (0.52, 0.035, 0.025, 1), 0.33, 1.2)
black = plastic("pla_black", (0.018, 0.018, 0.018, 1), 0.5, 0.55, 0.3)
crane_m = plastic("pla_crane", (0.03, 0.03, 0.032, 1), 0.45, 0.0, 0.3)
mats = {"skycrane": crane_m, "body": white, "lid": red, "mast_head": red, "mast_post": white, "dish": white, "rtg": white, "wheel": black}
for f in sorted(glob.glob(os.path.join(asm_dir, "*.stl"))):
    bpy.ops.wm.stl_import(filepath=f)
    o = bpy.context.selected_objects[0]
    o.data.materials.append(mats[os.path.basename(f)[3:-4]])
    o.scale = (0.01,) * 3
    o.location.z += LIFT

# ------------------------------------------------------------------ OLED screen behind the front window
m, n, l, b = nodes_mat("oled")
b.inputs["Base Color"].default_value = (0.005, 0.005, 0.006, 1); b.inputs["Roughness"].default_value = 0.05
tc = n.new("ShaderNodeTexCoord"); wv = n.new("ShaderNodeTexWave"); wv.wave_type = "BANDS"; wv.bands_direction = "Y"
wv.inputs["Scale"].default_value = 3.0; wv.inputs["Distortion"].default_value = 0
l.new(tc.outputs["UV"], wv.inputs["Vector"])
th = n.new("ShaderNodeMath"); th.operation = "GREATER_THAN"; th.inputs[1].default_value = -1.0
l.new(wv.outputs["Fac"], th.inputs[0])
em = n.new("ShaderNodeMath"); em.operation = "MULTIPLY"; em.inputs[1].default_value = 0.6; l.new(th.outputs[0], em.inputs[0])
b.inputs["Emission Color"].default_value = (0.05, 0.30, 0.38, 1); l.new(em.outputs[0], b.inputs["Emission Strength"])
bpy.ops.mesh.primitive_plane_add(size=1, location=(0.80, 0.247, 0.27 + LIFT), rotation=(0, math.radians(90), 0))
s = bpy.context.object; s.scale = (0.19, 0.27, 1); s.data.materials.append(m)

# thruster plumes (only when the crane is in the shot)
if LIFT > 0:
    plume = bpy.data.materials.new("plume"); plume.use_nodes = True
    pn, pl = plume.node_tree.nodes, plume.node_tree.links
    pn.remove(pn["Principled BSDF"])
    emi = pn.new("ShaderNodeEmission"); emi.inputs["Color"].default_value = (1.0, 0.62, 0.35, 1); emi.inputs["Strength"].default_value = 0.7
    tr = pn.new("ShaderNodeBsdfTransparent")
    lw = pn.new("ShaderNodeLayerWeight"); lw.inputs["Blend"].default_value = 0.3
    mixs = pn.new("ShaderNodeMixShader")
    pl.new(lw.outputs["Facing"], mixs.inputs["Fac"])
    pl.new(emi.outputs[0], mixs.inputs[1]); pl.new(tr.outputs[0], mixs.inputs[2])
    pl.new(mixs.outputs[0], pn["Material Output"].inputs["Surface"])
    ROOF = 0.555 + 0.025                                   # (OUT_Z + LID_T) / 100
    cz = ROOF + 0.50 - 0.15 + LIFT                         # nozzle exits
    for px in (-0.01, 0.866 + 0.01):
        for py in (-0.13, 0.614 + 0.13):
            for dx in (-0.036, 0.036):
                bpy.ops.mesh.primitive_cone_add(radius1=0.03, radius2=0.004, depth=0.10, location=(px + dx, py, cz - 0.05))
                c = bpy.context.object; c.data.materials.append(plume)
            bpy.ops.object.light_add(type="POINT", location=(px, py, cz - 0.08))
            bpy.context.object.data.energy = 2.5; bpy.context.object.data.color = (1.0, 0.6, 0.3)

# ------------------------------------------------------------------ terrain
cx, cy = 0.43, 0.31
WHEEL_Y = (-0.076, 0.69)           # wheel centre lanes (y)
SIZE = 16.0
bm = bmesh.new()
bmesh.ops.create_grid(bm, x_segments=RES, y_segments=RES, size=SIZE / 2)
track_layer = bm.verts.layers.float.new("track")

craters = [(3.2, 2.4, 1.1, 0.16), (-2.5, 4.0, 1.6, 0.22), (5.5, -1.5, 0.8, 0.10), (1.4, 5.8, 2.2, 0.28),
           (-4.5, -0.5, 1.2, 0.15), (2.4, -2.6, 0.5, 0.06), (7.0, 4.5, 1.8, 0.2), (-1.2, 2.6, 0.35, 0.04)]
for _ in range(220):                               # power-law small craters
    r = 0.05 * (1 / random.random()) ** 0.55
    r = min(r, 0.9)
    x, y = random.uniform(-7.5, 8.0), random.uniform(-7.5, 8.0)
    if math.hypot(x - cx, y - cy) < 1.1 + r:
        continue
    craters.append((x, y, r, r * random.uniform(0.12, 0.22)))


def track_y(x, lane):
    return lane + 0.05 * math.sin((x - 0.4) * 0.9)          # gentle curve behind the rover


for v in bm.verts:
    x, y = v.co.x + cx, v.co.y + cy
    v.co.x, v.co.y = x, y
    d = math.hypot(x - cx, y - cy)
    fade = min(1.0, max(0.0, (d - 0.8) / 1.8))
    h = 0.12 * noise.fractal(Vector((x * 0.3, y * 0.3, 0.0)), 1.0, 2.0, 6) * fade
    h += 0.010 * noise.fractal(Vector((x * 4, y * 4, 1.3)), 0.8, 2.1, 4)
    h += 0.0025 * noise.noise(Vector((x * 25, y * 25, 2.1)))
    for kx, ky, r, depth in craters:
        rr = math.hypot(x - kx, y - ky) / r
        if rr < 1.0:
            h -= depth * (1 - rr * rr) ** 1.2
        elif rr < 1.6:
            h += depth * 0.3 * (1 - (rr - 1) / 0.6) ** 2
    # rover wheel tracks behind the rover (rover drove +x)
    t = 0.0
    if x < 0.05 and x > -9:
        for lane in WHEEL_Y:
            dy = abs(y - track_y(x, lane))
            if dy < 0.07:
                w = (1 - (dy / 0.07) ** 2)
                h -= 0.007 * w
                t = max(t, w)
            elif dy < 0.10:
                h += 0.0025 * (1 - (dy - 0.07) / 0.03)   # pushed-up berm
        t *= min(1.0, max(0.0, (0.05 - x) / 0.15))
    v[track_layer] = t
    v.co.z = GROUND_Z + h
me = bpy.data.meshes.new("terrain")
bm.to_mesh(me); bm.free()
terrain = bpy.data.objects.new("terrain", me); sc.collection.objects.link(terrain)
for p in me.polygons:
    p.use_smooth = True

far_bm = bmesh.new()
bmesh.ops.create_grid(far_bm, x_segments=260, y_segments=260, size=60)
for v in far_bm.verts:
    x, y = v.co.x + cx, v.co.y + cy
    v.co.x, v.co.y = x, y
    d = math.hypot(x - cx, y - cy)
    rise = min(1.0, max(0.0, (d - 9.0) / 25.0))
    v.co.z = GROUND_Z - 0.35 + rise * (2.2 + 2.0 * noise.fractal(Vector((x * 0.05, y * 0.05, 4.0)), 1.0, 2.0, 6)) \
             + 0.25 * noise.fractal(Vector((x * 0.4, y * 0.4, 7.0)), 1.0, 2.0, 5) * min(1.0, d / 12)
far_me = bpy.data.meshes.new("far"); far_bm.to_mesh(far_me); far_bm.free()
far = bpy.data.objects.new("far", far_me); sc.collection.objects.link(far)
for p in far_me.polygons:
    p.use_smooth = True

# regolith shader: albedo variation + fine grain bump + tread pattern in the tracks
tm, n, l, b = nodes_mat("regolith")
b.inputs["Roughness"].default_value = 1.0
b.inputs["Specular IOR Level"].default_value = 0.2
tc = n.new("ShaderNodeTexCoord")
n1 = n.new("ShaderNodeTexNoise"); n1.inputs["Scale"].default_value = 1.5; n1.inputs["Detail"].default_value = 6
n2 = n.new("ShaderNodeTexNoise"); n2.inputs["Scale"].default_value = 40; n2.inputs["Detail"].default_value = 10
for t_ in (n1, n2):
    l.new(tc.outputs["Object"], t_.inputs["Vector"])
cr = n.new("ShaderNodeValToRGB")
cr.color_ramp.elements[0].color = (0.13, 0.125, 0.12, 1); cr.color_ramp.elements[1].color = (0.40, 0.385, 0.365, 1)
mixf = n.new("ShaderNodeMath"); mixf.operation = "MULTIPLY_ADD"
l.new(n1.outputs["Fac"], mixf.inputs[0]); mixf.inputs[1].default_value = 0.7
addf = n.new("ShaderNodeMath"); addf.operation = "MULTIPLY"; addf.inputs[1].default_value = 0.6
l.new(n2.outputs["Fac"], addf.inputs[0]); l.new(addf.outputs[0], mixf.inputs[2])
l.new(mixf.outputs[0], cr.inputs["Fac"])
# darker, compacted soil inside the tracks
att = n.new("ShaderNodeAttribute"); att.attribute_name = "track"
dark = n.new("ShaderNodeMix"); dark.data_type = "RGBA"; dark.blend_type = "MULTIPLY"
dark.inputs["B"].default_value = (0.72, 0.72, 0.72, 1)
l.new(att.outputs["Fac"], dark.inputs["Factor"]); l.new(cr.outputs["Color"], dark.inputs["A"])
l.new(dark.outputs["Result"], b.inputs["Base Color"])
# micro bump: grain + pebbles
grain = n.new("ShaderNodeTexNoise"); grain.inputs["Scale"].default_value = 900; grain.inputs["Detail"].default_value = 4
vor = n.new("ShaderNodeTexVoronoi"); vor.inputs["Scale"].default_value = 160
l.new(tc.outputs["Object"], grain.inputs["Vector"]); l.new(tc.outputs["Object"], vor.inputs["Vector"])
vinv = n.new("ShaderNodeMath"); vinv.operation = "SUBTRACT"; vinv.inputs[0].default_value = 1.0
l.new(vor.outputs["Distance"], vinv.inputs[1])
vp = n.new("ShaderNodeMath"); vp.operation = "POWER"; vp.inputs[1].default_value = 6
l.new(vinv.outputs[0], vp.inputs[0])
hsum = n.new("ShaderNodeMath"); hsum.operation = "ADD"
l.new(grain.outputs["Fac"], hsum.inputs[0]); l.new(vp.outputs[0], hsum.inputs[1])
# chevron tread imprint, only where the "track" attribute is set
sepo = n.new("ShaderNodeSeparateXYZ"); l.new(tc.outputs["Object"], sepo.inputs[0])
tread = n.new("ShaderNodeMath"); tread.operation = "MULTIPLY"; tread.inputs[1].default_value = 2 * math.pi / 0.067
l.new(sepo.outputs["X"], tread.inputs[0])
tsin = n.new("ShaderNodeMath"); tsin.operation = "SINE"; l.new(tread.outputs[0], tsin.inputs[0])
tsq = n.new("ShaderNodeMath"); tsq.operation = "GREATER_THAN"; tsq.inputs[1].default_value = 0.2
l.new(tsin.outputs[0], tsq.inputs[0])
tmask = n.new("ShaderNodeMath"); tmask.operation = "MULTIPLY"
l.new(tsq.outputs[0], tmask.inputs[0]); l.new(att.outputs["Fac"], tmask.inputs[1])
tsc = n.new("ShaderNodeMath"); tsc.operation = "MULTIPLY"; tsc.inputs[1].default_value = 1.5
l.new(tmask.outputs[0], tsc.inputs[0])
hall = n.new("ShaderNodeMath"); hall.operation = "ADD"
l.new(hsum.outputs[0], hall.inputs[0]); l.new(tsc.outputs[0], hall.inputs[1])
bump = n.new("ShaderNodeBump"); bump.inputs["Strength"].default_value = 0.6; bump.inputs["Distance"].default_value = 0.004
l.new(hall.outputs[0], bump.inputs["Height"]); l.new(bump.outputs["Normal"], b.inputs["Normal"])
terrain.data.materials.append(tm)
far.data.materials.append(tm)

# ------------------------------------------------------------------ rocks and pebbles (linked duplicates)
rock_m, n, l, b = nodes_mat("rock")
b.inputs["Roughness"].default_value = 0.92
rn = n.new("ShaderNodeTexNoise"); rn.inputs["Scale"].default_value = 30; rn.inputs["Detail"].default_value = 10
rr_ = n.new("ShaderNodeValToRGB"); rr_.color_ramp.elements[0].color = (0.10, 0.10, 0.095, 1)
rr_.color_ramp.elements[1].color = (0.26, 0.255, 0.24, 1)
l.new(rn.outputs["Fac"], rr_.inputs["Fac"]); l.new(rr_.outputs["Color"], b.inputs["Base Color"])
rn.inputs["Scale"].default_value = 12
rb = n.new("ShaderNodeBump"); rb.inputs["Strength"].default_value = 1.0
l.new(rn.outputs["Fac"], rb.inputs["Height"]); l.new(rb.outputs["Normal"], b.inputs["Normal"])

protos = []
for i in range(8):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=5, radius=1.0, location=(0, 0, -50))
    p = bpy.context.object
    for v in p.data.vertices:
        k = 1 + 0.30 * noise.fractal(v.co * 1.1 + Vector((i * 7.1, i * 3.3, 0)), 0.75, 2.0, 7) + 0.06 * noise.noise(v.co * 6 + Vector((i, 0, 0)))
        v.co *= k
        if v.co.z < -0.2:
            v.co.z = -0.2 + (v.co.z + 0.2) * 0.3                   # flat-ish bottoms sit in the soil
    for f_ in p.data.polygons:
        f_.use_smooth = True
    p.data.materials.append(rock_m)
    protos.append(p)


def place(x, y, s):
    o = random.choice(protos).copy(); sc.collection.objects.link(o)
    o.scale = (s, s * random.uniform(0.7, 1.2), s * random.uniform(0.45, 0.8))
    o.rotation_euler = (random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), random.uniform(0, 6.3))
    o.location = (x, y, GROUND_Z - s * 0.25 + 0.12 * noise.fractal(Vector((x * 0.3, y * 0.3, 0)), 1.0, 2.0, 6)
                  * min(1.0, max(0.0, (math.hypot(x - cx, y - cy) - 0.8) / 1.8)))


for i in range(900):                                                  # pebbles near the camera and rover
    a = random.uniform(0, 2 * math.pi); d = 0.25 + 3.5 * random.random() ** 1.6
    x, y = cx + d * math.cos(a), cy + d * math.sin(a)
    if -0.05 < x < 0.95 and -0.15 < y < 0.78:
        continue
    place(x, y, random.uniform(0.004, 0.018))
for i in range(70):                                                   # rocks
    a = random.uniform(0, 2 * math.pi); d = random.uniform(1.0, 7.0)
    x, y = cx + d * math.cos(a), cy + d * math.sin(a)
    place(x, y, random.uniform(0.03, 0.14) * (1 + d / 5))

# ------------------------------------------------------------------ sky: black, a few faint stars, Earth with a phase + atmosphere rim
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; wn = w.node_tree
bg = wn.nodes["Background"]
st = wn.nodes.new("ShaderNodeTexVoronoi"); st.inputs["Scale"].default_value = 700
sr = wn.nodes.new("ShaderNodeValToRGB"); sr.color_ramp.elements[0].color = (1, 1, 1, 1)
sr.color_ramp.elements[1].position = 0.03; sr.color_ramp.elements[1].color = (0, 0, 0, 1)
wn.links.new(st.outputs["Distance"], sr.inputs["Fac"]); wn.links.new(sr.outputs["Color"], bg.inputs["Color"])
bg.inputs["Strength"].default_value = 1.2

bpy.ops.mesh.primitive_uv_sphere_add(segments=128, ring_count=64, radius=1.3, location=(0, 30, 6))
earth = bpy.context.object
for p in earth.data.polygons:
    p.use_smooth = True
em_, n, l, b = nodes_mat("earth")
et = n.new("ShaderNodeTexNoise"); et.inputs["Scale"].default_value = 1.6; et.inputs["Detail"].default_value = 12
et.inputs["Roughness"].default_value = 0.62
er = n.new("ShaderNodeValToRGB"); er.color_ramp.interpolation = "LINEAR"
er.color_ramp.elements[0].position = 0.52; er.color_ramp.elements[0].color = (0.010, 0.045, 0.16, 1)
er.color_ramp.elements[1].position = 0.56; er.color_ramp.elements[1].color = (0.20, 0.22, 0.10, 1)
cl = n.new("ShaderNodeTexNoise"); cl.inputs["Scale"].default_value = 4.5; cl.inputs["Detail"].default_value = 14
cl.inputs["Distortion"].default_value = 1.5
crr = n.new("ShaderNodeValToRGB"); crr.color_ramp.elements[0].position = 0.5; crr.color_ramp.elements[1].position = 0.68
mx = n.new("ShaderNodeMix"); mx.data_type = "RGBA"; mx.inputs["B"].default_value = (0.92, 0.93, 0.95, 1)
l.new(et.outputs["Fac"], er.inputs["Fac"]); l.new(cl.outputs["Fac"], crr.inputs["Fac"])
l.new(crr.outputs["Color"], mx.inputs["Factor"]); l.new(er.outputs["Color"], mx.inputs["A"])
l.new(mx.outputs["Result"], b.inputs["Base Color"]); b.inputs["Roughness"].default_value = 0.7
# thin blue atmosphere glow on the rim
fr = n.new("ShaderNodeLayerWeight"); fr.inputs["Blend"].default_value = 0.12
rp = n.new("ShaderNodeMath"); rp.operation = "POWER"; rp.inputs[1].default_value = 3.0
l.new(fr.outputs["Facing"], rp.inputs[0])
rs = n.new("ShaderNodeMath"); rs.operation = "MULTIPLY"; rs.inputs[1].default_value = 1.2; l.new(rp.outputs[0], rs.inputs[0])
b.inputs["Emission Color"].default_value = (0.25, 0.5, 1.0, 1); l.new(rs.outputs[0], b.inputs["Emission Strength"])
earth.data.materials.append(em_)

# ------------------------------------------------------------------ light: one hard low sun, no atmosphere
SUN_FROM = Vector((1.0, 0.45, 0.20)).normalized()
bpy.ops.object.light_add(type="SUN")
sun = bpy.context.object; sun.rotation_euler = (-SUN_FROM).to_track_quat("-Z", "Y").to_euler()
sun.data.energy = 9.0; sun.data.angle = math.radians(0.53); sun.data.color = (1.0, 0.98, 0.95)
bpy.ops.object.light_add(type="SUN")                                   # faint earthshine / ground bounce
fill = bpy.context.object; fill.rotation_euler = (-Vector((-0.3, -1.0, 0.5))).to_track_quat("-Z", "Y").to_euler()
fill.data.energy = 0.18; fill.data.color = (0.75, 0.85, 1.0)

# ------------------------------------------------------------------ camera: low, slightly telephoto
target = Vector((0.40, 0.30, 0.30))
pos = Vector((3.1, -2.05, 0.30))
if LIFT > 0:
    target = Vector((0.42, 0.30, 0.42 + LIFT * 0.8)); pos = Vector((3.9, -2.6, 0.32))
bpy.ops.object.camera_add(location=pos)
cam = bpy.context.object; sc.camera = cam
cam.rotation_euler = (target - pos).to_track_quat("-Z", "Y").to_euler()
cam.data.lens = 50 if LIFT == 0 else 44; cam.data.sensor_width = 36
cam.data.dof.use_dof = True; cam.data.dof.focus_distance = (Vector((0.75, 0.2, 0.25 + LIFT)) - pos).length
cam.data.dof.aperture_fstop = 5.6
fwd = (target - pos).normalized(); right = fwd.cross(Vector((0, 0, 1))).normalized()
earth.location = pos + fwd * 32 - right * 7.2 + Vector((0, 0, 5.4))
# sunlit side of Earth faces the sun -> natural phase

sc.render.engine = "CYCLES"; sc.cycles.samples = SAMPLES; sc.cycles.use_denoising = True
sc.cycles.max_bounces = 6
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "METAL"; prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    sc.cycles.device = "GPU"
except Exception as e:
    print("GPU fallback", e)
sc.render.resolution_x, sc.render.resolution_y = W, H
sc.view_settings.view_transform = "AgX"
try:
    sc.view_settings.look = "AgX - Medium High Contrast"
except Exception:
    pass
sc.view_settings.exposure = 0.15
sc.render.filepath = out_png
bpy.ops.render.render(write_still=True)
