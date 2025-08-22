import trimesh
import pathlib
import json
import bpy, sys

DATA_ROOT   = pathlib.Path("/datasets/scene_datasets/replica_cad_dataset/")                    # HF repo root
SCENE_DIR   = DATA_ROOT / "configs" / "scenes"

demo_scene = SCENE_DIR / "apt_0.scene_instance.json"
inst = json.loads(demo_scene.read_text())
stage_name = inst["stage_instance"]["template_name"]   # 'stages/apt_0_stage'
stage_glb  = DATA_ROOT / f"{stage_name}.glb"

in_path = sys.argv[-2]; 
out_path = sys.argv[-1]

bpy.ops.import_scene.gltf(filepath=in_path)

# 모든 Object 선택
for obj in bpy.context.scene.objects:
    bpy.context.view_layer.objects.active = obj
    if obj.type != 'MESH': continue

    # 1) Apply transforms
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    # 2) Recalculate normals
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.editmode_toggle()

    # 3) Remove doubles / degenerate
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.merge_by_distance(distance=1e-5)
    bpy.ops.mesh.delete_loose()
    bpy.ops.object.editmode_toggle()

    # 4) (Stage 등) Solidify for thickness
    if "floor" in obj.name.lower() or "wall" in obj.name.lower():
        mod = obj.modifiers.new("Solidify","SOLIDIFY")
        mod.thickness = 0.03
        bpy.ops.object.modifier_apply(modifier=mod.name)

# (옵션) Join into one collision mesh
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.join()

bpy.ops.export_scene.gltf(filepath=out_path, export_format='GLB', export_apply=True)
