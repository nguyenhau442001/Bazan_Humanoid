"""Blender side of render.py: reads the manifest (STL files, colours, cameras) and renders each view."""
import json
import sys

import bpy  # noqa: I001
from mathutils import Vector

COL = {"white": "#F3F5F8", "navy": "#2A3D5E", "servo": "#3D7BE0", "horn": "#E9EDF2", "cap": "#22324E",
       "sole": "#1E3557", "new": "#F2711C"}


def hexrgb(h, a=1.0):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)) + (a,)


def mat(name, color, xray=False):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = hexrgb(color)
    b.inputs["Roughness"].default_value = 0.45
    if xray and name != "servo":
        b.inputs["Alpha"].default_value = 0.18
        m.blend_method = "BLEND" if hasattr(m, "blend_method") else None
    return m


def render(view, samples, out):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    mats = {k: mat(k, v, view["xray"]) for k, v in COL.items()}
    for f, look in view["files"]:
        before = set(bpy.data.objects)
        bpy.ops.wm.stl_import(filepath=f)
        ob = (set(bpy.data.objects) - before).pop()
        ob.scale = (0.001, 0.001, 0.001)
        ob.data.materials.clear()
        ob.data.materials.append(mats[look])
        for p in ob.data.polygons:
            p.use_smooth = False
    w = bpy.data.worlds.new("w")
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs["Color"].default_value = hexrgb("#E7ECF2")
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.35
    sc.world = w
    lights = (("key", (0.5, -0.4, 0.6), 30), ("fill", (0.4, 0.7, 0.3), 10), ("rim", (-0.6, 0.2, 0.4), 14))
    for name, loc, power in lights:
        ld = bpy.data.lights.new(name, "AREA")
        ld.size = 0.6
        ld.energy = power
        ob = bpy.data.objects.new(name, ld)
        sc.collection.objects.link(ob)
        ob.location = loc
        ob.rotation_euler = (Vector((0, 0, -0.05)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    cd = bpy.data.cameras.new("cam")
    if view["ortho"]:
        cd.type = "ORTHO"
        cd.ortho_scale = view["ortho"]
    else:
        cd.lens = 85
    cam = bpy.data.objects.new("cam", cd)
    sc.collection.objects.link(cam)
    cam.location = view["cam"]
    cam.rotation_euler = (Vector(view["target"]) - Vector(view["cam"])).to_track_quat("-Z", "Y").to_euler()
    sc.camera = cam
    r = sc.render
    r.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    r.resolution_x, r.resolution_y = 900, 1100
    r.film_transparent = False
    sc.view_settings.view_transform = "AgX"
    r.image_settings.file_format = "JPEG"
    r.image_settings.quality = 88
    r.filepath = f"{out}/{view['name']}.jpg"
    bpy.ops.render.render(write_still=True)


def main():
    m = json.loads(open(sys.argv[1]).read())
    for v in m["views"]:
        render(v, m["samples"], m["out"])


if __name__ == "__main__":
    main()
