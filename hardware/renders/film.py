import bpy, sys, math
from mathutils import Vector
R="/Users/alpme/Projects/hackathons/nasa space apps 2026"
argv=sys.argv[sys.argv.index("--")+1:]
OUT=argv[0]; FR=argv[1] if len(argv)>1 else "all"
LIFT=1.0
src=open(R+"/presentation/blender_hero_real.py").read()
src=src[:src.index("# ------------------------------------------------------------------ camera")]
sys.argv=["x","--",R+"/enclosure/assembly_descent",OUT,"960","540","16","300",str(LIFT)]
g={"__name__":"__main__"}; exec(compile(src,"hero","exec"),g)
sc=bpy.context.scene; cx,cy=g["cx"],g["cy"]
objs=list(sc.objects)
master=bpy.data.objects.new("lander",None); sc.collection.objects.link(master)
crane=bpy.data.objects.new("crane",None); sc.collection.objects.link(crane); crane.parent=master
def is_crane(o):
    if o.name.startswith("12_"): return True
    if o.type=="MESH" and o.data.materials and o.data.materials[0].name=="plume": return True
    if o.type=="LIGHT" and o.data.type=="POINT": return True
    return False
for o in objs:
    rover_part = o.type=="MESH" and (o.name[:2].isdigit() or (o.data.materials and o.data.materials[0].name=="oled"))
    if rover_part or is_crane(o):
        o.parent = crane if is_crane(o) else master
# ground height under Z1 / Z2
dg=bpy.context.evaluated_depsgraph_get()
master.location.z=500; bpy.context.view_layer.update(); dg=bpy.context.evaluated_depsgraph_get()
_gzc={}
def gz(x,y):
    k=(round(x,3),round(y,3))
    if k not in _gzc:
        hit,loc,*_=sc.ray_cast(dg,Vector((x,y,5)),Vector((0,0,-1))); _gzc[k]=loc.z if hit else 0.0
    return _gzc[k]
Z1=(cx,cy)
OFF1=Vector((3.6,-2.7,2.1)); OFF2=Vector((2.9,-2.2,1.55))
cam_d=bpy.data.cameras.new("cam"); cam=bpy.data.objects.new("cam",cam_d); sc.collection.objects.link(cam); sc.camera=cam
z1v=Vector((Z1[0],Z1[1],gz(*Z1))); cam.location=z1v+OFF1; cam.rotation_euler=(-OFF1).to_track_quat("-Z","Y").to_euler()
bpy.context.view_layer.update()
rgt=cam.matrix_world.to_3x3().col[0].copy(); rgt.z=0; rgt.normalize()
Z2=(Z1[0]+1.45*rgt.x, Z1[1]+1.45*rgt.y)
print("RIGHT",rgt,"Z2",Z2)
dz2 = gz(*Z2)-gz(*Z1)
print("GZ",gz(*Z1),gz(*Z2))
# boulder on Z1
p=g["protos"][2].copy(); sc.collection.objects.link(p); p.scale=(0.22,0.18,0.15); p.location=(cx+0.15,cy+0.05,gz(cx,cy)+0.03)
# zone rings
def ring(name,xy,col):
    bpy.ops.mesh.primitive_torus_add(major_radius=0.62,minor_radius=0.018,location=(xy[0],xy[1],gz(*xy)+0.03))
    r=bpy.context.object; m=bpy.data.materials.new(name); m.use_nodes=True; b=m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value=(0,0,0,1); b.inputs["Emission Color"].default_value=col; b.inputs["Emission Strength"].default_value=6
    r.data.materials.append(m); return b
b1=ring("z1",Z1,(1.0,0.85,0.2,1)); r1=bpy.context.object; ring("z2",Z2,(0.2,1.0,0.55,1)); r2=bpy.context.object
for r,f0 in ((r1,150),(r2,165)):
    r.scale=(0.001,0.001,0.001); r.keyframe_insert("scale",frame=f0)
    r.scale=(1.08,1.08,1.08); r.keyframe_insert("scale",frame=f0+14)
    r.scale=(1,1,1); r.keyframe_insert("scale",frame=f0+20)
ec=b1.inputs["Emission Color"]
for f,c in ((285,(1.0,0.85,0.2,1)),(300,(1.0,0.08,0.05,1))):
    ec.default_value=c; ec.keyframe_insert("default_value",frame=f)
master.location=(0,0,0)
# lander keys (offsets from rest pose which already sits LIFT above Z1)
H=lambda z: z
G0=g.get("GROUND_Z",0.0); print("G0",G0)
def off(xy,h): return (xy[0]-cx, xy[1]-cy, gz(*xy)-G0-LIFT+h)
keys=[(1,off((Z1[0]-2.0,Z1[1]+1.0),7.0)),(240,off(Z1,2.0)),(300,off(Z1,0.85)),(345,off(Z1,0.8)),(470,off(Z2,0.6)),(540,off(Z2,0.0)),(870,off(Z2,0.0))]
for f,v in keys:
    master.location=v; master.keyframe_insert("location",frame=f)
# crane releases and climbs away after touchdown (rover stays)
crane.location=(0,0,0); crane.keyframe_insert("location",frame=548)
crane.location=(0.5,0.3,4.0); crane.keyframe_insert("location",frame=680)
# camera: wide, slow push, sees both zones
mid=Vector(((Z1[0]+Z2[0])/2,(Z1[1]+Z2[1])/2,gz(*Z1)+0.45))
z2v=Vector((Z2[0],Z2[1],gz(*Z2)+0.35))
cam_d.lens=30; cam_d.sensor_width=36
tgt=bpy.data.objects.new("tgt",None); sc.collection.objects.link(tgt)
for f,v in ((1,mid+Vector((-1.2,0.8,1.6))),(240,mid),(480,mid),(600,z2v),(870,z2v)):
    tgt.location=v; tgt.keyframe_insert("location",frame=f)
cam.rotation_euler=(0,0,0)
con=cam.constraints.new("TRACK_TO"); con.target=tgt; con.track_axis="TRACK_NEGATIVE_Z"; con.up_axis="UP_Y"
for f,pos in ((1,mid+Vector((7.5,-5.6,6.5))),(240,mid+OFF1),(480,mid+OFF2),(600,z2v+Vector((2.4,-1.6,0.7))),(735,z2v+Vector((0.2,-2.7,0.5))),(870,z2v+Vector((-1.9,-1.9,0.6)))):
    cam.location=pos; cam.keyframe_insert("location",frame=f)
earth=[o for o in sc.objects if o.name.startswith("Sphere")]
if earth:
    pos=mid+Vector((5.5,-4.1,4.3)); fwd=(mid-pos).normalized(); right=fwd.cross(Vector((0,0,1))).normalized()
    earth[0].location=pos+fwd*32-right*11+Vector((0,0,10.5))
sc.frame_start,sc.frame_end=1,870; sc.render.fps=30
sc.render.engine="BLENDER_EEVEE"
try: sc.eevee.taa_render_samples=12
except Exception: pass
sc.render.resolution_x,sc.render.resolution_y=960,540
sc.view_settings.view_transform="AgX"; sc.view_settings.exposure=0.15
sc.render.image_settings.file_format="PNG"
sc.render.filepath=OUT+"/f_"
if FR=="all": bpy.ops.render.render(animation=True)
else:
    for f in map(int,FR.split(",")):
        sc.frame_set(f); sc.render.filepath=f"{OUT}/t_{f:04d}.png"; bpy.ops.render.render(write_still=True)
