import os
import bpy

root = 'C:/Users/lly/Desktop/Qlone/'

for dir_name in os.listdir(root):
    dir_path = os.path.join(root, dir_name)
    if not os.path.isdir(dir_path):
        # print(dir_path)
        continue

    print(dir_path)
    class_idx = dir_name
    for file in os.listdir(dir_path):
        if 'obj' in file:
            bpy.ops.import_scene.obj(filepath=os.path.join(dir_path, file))
            imported = bpy.context.selected_objects[0]
            bpy.ops.transform.resize(value=(0.002, 0.002, 0.002))
            bpy.ops.object.select_all(action="DESELECT")
            imported.select = True
            bpy.ops.export_scene.obj(filepath=os.path.join(dir_path, str(class_idx)+'.obj'), use_selection=True)
