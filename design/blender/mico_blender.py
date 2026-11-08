"""Build Mico v2.0 in Blender, save mico.blend and render the design boards.

    pip install bpy==5.2.3        # or run inside Blender: blender -b -P mico_blender.py -- --no-render
    python mico_blender.py                 # mico.blend + renders/*.png (Cycles, CPU)
    python mico_blender.py --no-render     # only mico.blend
    python mico_blender.py --samples 256   # cleaner renders

The rig: one empty per joint, nested like the URDF and placed at the same frames, with rotation
locked to the joint axis. Select e.g. "J l_shoulder_roll" in Blender and rotate it to pose Mico.
"""
import math
import sys
from pathlib import Path

import bpy  # noqa: I001  (bpy must come first: it registers bmesh and mathutils)
import numpy as np
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mico_parts as mp  # noqa: E402

MM = 0.001
args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
RENDER = "--no-render" not in args
SAMPLES = int(args[args.index("--samples") + 1]) if "--samples" in args else 96
OUT = HERE / "renders"


# ---------------------------------------------------------------- materials
def hexrgb(h, a=1.0):
    h = h.lstrip("#")
    srgb = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    lin = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb]
    return (*lin, a)


def principled(name, color, rough=0.4, coat=0.0, metal=0.0, emit=None, strength=0.0, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = hexrgb(color)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Coat Weight"].default_value = coat
    b.inputs["Coat Roughness"].default_value = 0.08
    if emit:
        b.inputs["Emission Color"].default_value = hexrgb(emit)
        b.inputs["Emission Strength"].default_value = strength
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
    m.diffuse_color = hexrgb(emit or color)
    return m


def thin_glass(name, tint="#EAF4FF"):
    """Thin clear dome: transparent + fresnel gloss, no refraction (a 0.8 mm wall barely bends light)."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    tr.inputs["Color"].default_value = hexrgb(tint)
    gl = nt.nodes.new("ShaderNodeBsdfGlossy")
    gl.inputs["Roughness"].default_value = 0.03
    fr = nt.nodes.new("ShaderNodeLayerWeight")
    fr.inputs["Blend"].default_value = 0.25
    mx = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(fr.outputs["Fresnel"], mx.inputs["Fac"])
    nt.links.new(tr.outputs[0], mx.inputs[1])
    nt.links.new(gl.outputs[0], mx.inputs[2])
    nt.links.new(mx.outputs[0], out.inputs["Surface"])
    m.diffuse_color = (0.85, 0.92, 1.0, 0.25)
    return m


def xray(name, color, opacity=0.18):
    """See-through shell for the internals board: tinted transparent with bright edges."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    df = nt.nodes.new("ShaderNodeBsdfDiffuse")
    df.inputs["Color"].default_value = hexrgb(color)
    lw = nt.nodes.new("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.35
    mth = nt.nodes.new("ShaderNodeMath")
    mth.operation = "MULTIPLY_ADD"
    mth.inputs[1].default_value = 1 - opacity
    mth.inputs[2].default_value = opacity
    mx = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(lw.outputs["Facing"], mth.inputs[0])
    nt.links.new(mth.outputs[0], mx.inputs["Fac"])
    nt.links.new(tr.outputs[0], mx.inputs[1])
    nt.links.new(df.outputs[0], mx.inputs[2])
    nt.links.new(mx.outputs[0], out.inputs["Surface"])
    return m


def make_materials():
    M = {
        "white": principled("Shell white (matte PLA)", "#F6F7F9", 0.42, coat=0.12),
        "orange": principled("Signal orange", "#F05A00", 0.38, coat=0.15),   # reads as #FF7A1A under AgX
        "grey": principled("Panel grey", "#C9D1DC", 0.45),
        "joint": principled("Joint navy", "#2A3D5E", 0.48, coat=0.1),
        "visor": principled("Visor navy", "#13233D", 0.1, coat=0.8),
        "window": principled("Camera window", "#06080C", 0.04, coat=1.0),
        "pupil": principled("Pupil navy", "#0E1B30", 0.3),
        "eye_glow": principled("Eye glow", "#2E86B0", 0.4, emit="#5BC8F5", strength=0.9),
        "navy": principled("Sole navy", "#1E3557", 0.55),
        "screen": principled("Screen", "#13233D", 0.12, coat=0.6),
        "cyan": principled("Eye cyan", "#5BC8F5", 0.3, emit="#5BC8F5", strength=2.5),
        "glint": principled("Glint", "#FFFFFF", 0.2, emit="#FFFFFF", strength=8.0),
        "red": principled("Flag red", "#DA251D", 0.4),
        "yellow": principled("Star yellow", "#FFCD00", 0.35, emit="#FFCD00", strength=0.3),
        "blush": principled("Blush", "#FF9C9C", 0.6, alpha=0.75),
        "lens": principled("Lens", "#08090C", 0.05, coat=1.0),
        "glass": thin_glass("Helmet PETG"),
        # internals
        "servo": principled("Servo MG92B", "#3D7BE0", 0.5),
        "mg90s": principled("Servo MG90S", "#4A5578", 0.5),
        "sg90": principled("Servo SG90", "#76B6F2", 0.5),
        "battery": principled("Battery 2S", "#F2735A", 0.5),
        "pcb": principled("PCB", "#3FAF85", 0.45),
        "extra": principled("Sensors / brackets", "#A98BE0", 0.5),
        "hidden": principled("Wiring", "#E8C547", 0.6),
        "floor": principled("Floor", "#EEF2F7", 0.6),
    }
    M["xray_white"] = xray("X-ray shell", "#DCE4EE", 0.06)
    M["xray_orange"] = xray("X-ray accent", "#F57A1F", 0.12)
    return M


# ---------------------------------------------------------------- scene helpers
def mesh_obj(name, V, F, mat, coll, smooth=True):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in V], [], [tuple(int(i) for i in f) for f in F])
    me.validate()
    if smooth:
        me.shade_smooth()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def box_mesh(size):
    x, y, z = (s / 2 for s in size)
    V = [(sx * x, sy * y, sz * z) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return V, F


def scaled(V):
    return [tuple(np.asarray(v) * MM) for v in V]


# ---------------------------------------------------------------- rig
def build_rig(coll):
    """One empty per link, nested like the URDF; joints rotate about their own axis only."""
    E = {}

    def rec(b, parent):
        e = bpy.data.objects.new(f"J {b.name}", None)
        e.empty_display_type = "ARROWS" if b.joint else "CUBE"
        e.empty_display_size = 0.012
        coll.objects.link(e)
        e.location = Vector(mp.ORIGIN[b.name]) * MM
        if parent:
            e.parent = parent
            e.matrix_parent_inverse = parent.matrix_world.inverted()
        if b.joint:
            ax = [abs(a) for a in b.joint.axis].index(1)
            e.lock_rotation = [i != ax for i in range(3)]
            e["joint_axis"] = list(b.joint.axis)
            e["range_deg"] = list(b.joint.range_deg)
            e["servo"] = b.joint.servo
        e.lock_location = (True, True, True)
        E[b.name] = e
        bpy.context.view_layer.update()
        for c in b.children:
            rec(c, e)
    rec(mp.gen_robot.ROBOT, None)
    return E


def attach(ob, link_empty):
    bpy.context.view_layer.update()
    ob.parent = link_empty
    ob.matrix_parent_inverse = link_empty.matrix_world.inverted()


def pose(E, q):
    """q: {joint name: degrees}. Rotation about the joint axis (sign as in the URDF)."""
    for b in mp.gen_robot.walk(mp.gen_robot.ROBOT):
        if not b.joint:
            continue
        e = E[b.name]
        e.rotation_euler = (0, 0, 0)
        ax = [abs(a) for a in b.joint.axis].index(1)
        r = list(e.rotation_euler)
        r[ax] = math.radians(q.get(b.name, 0.0)) * b.joint.axis[ax]
        e.rotation_euler = r
    bpy.context.view_layer.update()


# ---------------------------------------------------------------- build
def build():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.length_unit = "MILLIMETERS"
    M = make_materials()

    root = sc.collection
    c_rig = bpy.data.collections.new("Rig (joints)")
    c_shell = bpy.data.collections.new("Shell")
    c_deco = bpy.data.collections.new("Paint and lights")
    c_int = bpy.data.collections.new("Internals")
    c_studio = bpy.data.collections.new("Studio")
    for c in (c_rig, c_shell, c_deco, c_int, c_studio):
        root.children.link(c)
    E = build_rig(c_rig)

    objs = {}
    for p in mp.parts():
        V, F = mp.part_mesh(p)
        ob = mesh_obj(p.name, scaled(V), F, M[p.look], c_shell if p.mass else c_deco)
        ob["mass_model"] = str(p.mass)
        attach(ob, E[p.link])
        objs[p.name] = ob

    for name, link, look, V, F in mp.decals():                 # star, smile, ECG trace
        attach(mesh_obj(name, scaled(V), F, M[look], c_deco, smooth=name != "flag_star"), E[link])

    # internals: servos, battery, boards, sensors, brackets (hidden in the beauty renders)
    kind_mat = {"servo": None, "battery": "battery", "pcb": "pcb", "extra": "extra", "hidden": "hidden"}
    servo_mat = {"MG92B": "servo", "DS843MG": "servo", "MG90S": "mg90s", "SG90": "sg90"}
    servo_of = {b.joint.name: b.joint.servo for b in mp.gen_robot.walk(mp.gen_robot.ROBOT) if b.joint}
    for name, link, size, pos, grams, kind in mp.components():
        if kind == "servo":
            mat = M[servo_mat[servo_of.get(name.replace("_servo", ""), "MG92B")]]
        else:
            mat = M[kind_mat.get(kind, "extra")]
        if name.endswith("_bracket"):
            continue                                   # brackets are a mass allowance, not a shape
        V, F = box_mesh(size)
        ob = mesh_obj(name, scaled(np.add(V, pos)), F, mat, c_int, smooth=False)
        ob["grams"] = grams
        bev = ob.modifiers.new("bevel", "BEVEL")
        bev.width = 0.0008
        bev.segments = 2
        attach(ob, E[link])
    c_int.hide_render = True
    bpy.context.view_layer.layer_collection.children["Internals"].hide_viewport = False

    studio(sc, M, c_studio)
    return sc, E, M, objs, c_int


def studio(sc, M, coll):
    bpy.ops.mesh.primitive_plane_add(size=3)
    fl = bpy.context.active_object
    fl.name = "floor (shadow catcher)"
    fl.data.materials.append(M["floor"])
    fl.is_shadow_catcher = True
    for c in fl.users_collection:
        c.objects.unlink(fl)
    coll.objects.link(fl)

    def area(name, loc, size, power, color="#FFFFFF", shadow=True):
        ld = bpy.data.lights.new(name, "AREA")
        ld.size = size
        ld.energy = power
        ld.color = hexrgb(color)[:3]
        ld.use_shadow = shadow
        ob = bpy.data.objects.new(name, ld)
        coll.objects.link(ob)
        ob.location = loc
        d = Vector((0, 0, 0.16)) - Vector(loc)
        ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        return ob
    area("key", (0.6, -0.5, 1.1), 0.7, 42, "#FFF4E8")
    area("fill", (0.5, 0.8, 0.8), 0.9, 12, "#E6F0FF", shadow=False)
    area("rim", (-0.7, 0.25, 0.85), 0.4, 34, "#DCEBFF", shadow=False)
    area("rim2", (-0.5, -0.6, 0.7), 0.4, 14, shadow=False)

    w = bpy.data.worlds.new("studio")
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = hexrgb("#DDE6F0")
    bg.inputs["Strength"].default_value = 0.35
    sc.world = w

    r = sc.render
    r.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = SAMPLES
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.02
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 8
    sc.cycles.transparent_max_bounces = 16
    sc.cycles.caustics_reflective = sc.cycles.caustics_refractive = False
    r.film_transparent = True
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGBA"
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Punchy"
    sc.view_settings.exposure = 0.0


def camera(sc, name, loc, target=(0, 0, 0.158), lens=None, ortho=None):
    cd = bpy.data.cameras.new(name)
    if ortho:
        cd.type = "ORTHO"
        cd.ortho_scale = ortho
    else:
        cd.lens = lens
    cd.clip_start = 0.01
    ob = bpy.data.objects.new(name, cd)
    sc.collection.objects.link(ob)
    ob.location = loc
    ob.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return ob


def set_face(E, M, expr):
    """Swap the face layers on the visor (eyes and mouth) for one of mp.EXPRESSIONS."""
    coll = bpy.data.collections["Paint and lights"]
    for ob in list(coll.objects):
        if ob.name.startswith(("eye_", "mouth")):
            bpy.data.objects.remove(ob, do_unlink=True)
    for name, link, look, V, F in mp.decals(expr):
        if name.startswith(("eye_", "mouth")):
            attach(mesh_obj(name, scaled(V), F, M[look], coll), E[link])


def render_faces(sc, E, M):
    """Front view of the visor for each expression (the OLED animation frames)."""
    pose(E, {})
    sc.camera = camera(sc, "cam_faces", (1.0, 0, 0.268), ortho=0.074, target=(0, 0, 0.268))
    sc.render.resolution_x, sc.render.resolution_y = 520, 420
    for expr in mp.EXPRESSIONS:
        set_face(E, M, expr)
        sc.render.filepath = str(OUT / f"face_{expr}.png")
        bpy.ops.render.render(write_still=True)
        print("rendered face", expr, flush=True)
    set_face(E, M, "neutral")


WAVE = {"l_shoulder_roll": 125, "l_elbow": -55, "l_shoulder_pitch": -15, "r_shoulder_roll": -8,
        "head_pitch": -6, "neck_yaw": -12, "r_hip_roll": -2, "l_hip_roll": 2}


def render_all(sc, E, M, objs, c_int, only=None):
    OUT.mkdir(exist_ok=True)
    d = 1.05
    shots = [
        ("hero", dict(loc=(d * math.cos(math.radians(-28)), d * math.sin(math.radians(-28)), 0.36), lens=80),
         WAVE, (1200, 1500)),
        ("front", dict(loc=(1.2, 0, 0.17), ortho=0.36, target=(0, 0, 0.162)), {}, (900, 1200)),
        ("side", dict(loc=(0, -1.2, 0.17), ortho=0.36, target=(0, 0, 0.162)), {}, (900, 1200)),
        ("back", dict(loc=(-1.2, 0, 0.17), ortho=0.36, target=(0, 0, 0.162)), {}, (900, 1200)),
        ("head", dict(loc=(0.42, -0.2, 0.33), lens=95, target=(0.0, 0, 0.270)), {"neck_yaw": -10}, (1000, 900)),
    ]
    DETAIL = {"l_shoulder_roll": 28, "l_elbow": -45, "l_shoulder_pitch": -25,
              "l_hip_pitch": -22, "l_knee": 38, "l_ankle_pitch": -16}
    shots += [
        ("detail_arm", dict(loc=(0.36, 0.30, 0.25), lens=120, target=(0.006, 0.052, 0.180)), DETAIL, (900, 900)),
        ("detail_leg", dict(loc=(0.36, -0.20, 0.17), lens=80, target=(0.0, 0.0, 0.088)), DETAIL, (900, 1000)),
        ("detail_foot", dict(loc=(0.22, -0.25, 0.10), lens=95, target=(0.008, -0.03, 0.018)), {}, (900, 700)),
    ]
    for name, cam, q, res in shots:
        if only and name not in only:
            continue
        pose(E, q)
        sc.camera = camera(sc, f"cam_{name}", **cam)
        sc.render.resolution_x, sc.render.resolution_y = res
        sc.render.filepath = str(OUT / f"{name}.png")
        bpy.ops.render.render(write_still=True)
        print("rendered", name, flush=True)

    if not only or "faces" in only:
        render_faces(sc, E, M)
    if only and "xray" not in only:
        return

    # x-ray: same 3/4 view, shell see-through, internals on
    pose(E, {})
    swap = {"white": "xray_white", "grey": "xray_white", "orange": "xray_orange", "joint": "xray_white",
            "visor": "xray_white"}
    for ob in objs.values():
        key = next((k for k, v in M.items() if ob.active_material == v), None)
        if key in swap:
            ob.active_material = M[swap[key]]
    for ob in bpy.data.collections["Paint and lights"].objects:
        ob.hide_render = True
    c_int.hide_render = False
    sc.camera = camera(sc, "cam_xray", (d * math.cos(math.radians(-35)), d * math.sin(math.radians(-35)), 0.30),
                       lens=100)
    sc.render.resolution_x, sc.render.resolution_y = 1200, 1500
    sc.render.filepath = str(OUT / "xray.png")
    bpy.ops.render.render(write_still=True)
    print("rendered xray", flush=True)


if __name__ == "__main__":
    sc, E, M, objs, c_int = build()
    pose(E, {})
    sc.camera = camera(sc, "cam_view", (0.9, -0.55, 0.35), lens=70)
    bpy.ops.wm.save_as_mainfile(filepath=str(HERE / "mico.blend"), compress=True)
    print("saved", HERE / "mico.blend", flush=True)
    if RENDER:
        only = args[args.index("--only") + 1].split(",") if "--only" in args else None
        render_all(sc, E, M, objs, c_int, only)
