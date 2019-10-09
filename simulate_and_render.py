"""
@file simulate_and_render.py
@copyright Software License Agreement (BSD License).
Copyright (c) 2017, Rutgers the State UniversplacePointLightity of New Jersey, New Brunswick,
All Rights Reserved. For a full description see the file named LICENSE.
Authors: Chaitanya Mitash, Kostas Bekris, Abdeslam Boularias.
"""

import sys, os
import bpy
import math, random
import numpy as np

# Verify if repository path is set in bashrc
if os.environ.get('PHYSIM_GENDATA') == None:
    print("Please set PHYSIM_GENDATA in bashrc!")
    sys.exit()

g_repo_path = os.environ['PHYSIM_GENDATA']
sys.path.append(g_repo_path)

from Environment import Shelf, Table, Light, Board
from ConfigParser import ConfigParser
from Camera import Camera

if __name__ == "__main__":

    ## read configuration file
    cfg = ConfigParser("config.yml")

    ## initialize resting surface
    env = cfg.getSurfaceType()
    if env == 'table':
        surface = Table('surface_models/table/table.obj')
    elif env == 'shelf':
        surface = Shelf('surface_models/shelf/shelf.obj')
    elif env == 'board':
        surface = Board('surface_models/board/board.obj')
    else:
        print("unknown env!")
        sys.exit()
    sPose = cfg.getSurfacePose()
    surface.setPose(sPose)

    ## initialize camera
    camIntrinsic = cfg.getCamIntrinsic()
    camExtrinsic = cfg.getCamExtrinsic()
    numViews = cfg.getNumViews()
    cam = Camera(camIntrinsic, camExtrinsic, numViews)

    ## initialize light
    pLight = Light()

    ## initialize objects
    objModelList = cfg.getObjModelList()
    objectlist = []
    for objFileName in objModelList:
        objFileName = str(objFileName)
        # objFileName='baishikele'
        # bpy.ops.import_scene.obj(filepath="../../obj_models/checkout/" + objFileName + "/" + objFileName + ".obj")
        bpy.ops.import_scene.obj(filepath="obj_models/checkout/" + objFileName + "/" + objFileName + ".obj")
        imported = bpy.context.selected_objects[0]
        objectlist.append(imported.name)
        print("imported:", imported.name)

    ## effect of illumination is currently disabled.
    for item in bpy.data.materials:
        print(item)
        # item.emit = 0.05
        item.use_shadeless = True

    num = 0
    numImages = cfg.getNumTrainingImages()
    while num < numImages:

        ## hide all objects  先把所有物体藏起来
        for obj in objectlist:
            bpy.data.objects[obj].hide = True    # 隐藏物体
            bpy.data.objects[obj].hide_render = True
            bpy.data.objects[obj].location[0] = 100.0
            # 用于缩放物体的代码
            # if bpy.data.objects[obj].dimensions[0] > 1:
            #     bpy.ops.transform.resize(value=(0.002, 0.002, 0.002))

        ## choose a random subset of objects for the scene
        sceneobjectlist = list(objectlist)
        minObjects = cfg.getMinObjectsScene()
        maxObjects = cfg.getMaxObjectsScene()
        numObjectsInScene = random.randint(minObjects, maxObjects)
        selectedobj = list()
        while len(selectedobj) < numObjectsInScene:
            index = random.randint(0, len(objectlist) - 1)
            if index in selectedobj:
                continue
            selectedobj.append(index)

        print("numObjectsInScene : ", numObjectsInScene)
        print("selected set is : ", selectedobj)

        # sample initial pose for each of the selected object
        for i in range(0, numObjectsInScene):
            index = selectedobj[i]
            shape_file = objectlist[index]
            sceneobjectlist[index] = shape_file
            # 把物体显示出来
            bpy.data.objects[shape_file].hide = False
            bpy.data.objects[shape_file].hide_render = False
            range_x = cfg.getRangeX()
            range_y = cfg.getRangeY()
            range_z = cfg.getRangeZ()
            bpy.data.objects[shape_file].location = (random.uniform(range_x[0], range_x[1]),
                                                     random.uniform(range_y[0], range_y[1]),
                                                     random.uniform(range_z[0], range_z[1]))

            # 随机采样欧拉角
            bpy.data.objects[shape_file].rotation_mode = 'XYZ'
            bpy.data.objects[shape_file].rotation_euler = (random.randint(0, 360) * 3.14 / 180.0,
                                                           random.randint(0, 360) * 3.14 / 180.0,
                                                           random.randint(0, 360) * 3.14 / 180.0)

            # 选中物体
            bpy.context.scene.objects.active = bpy.context.scene.objects[shape_file]
            bpy.context.scene.objects[shape_file].select = True    # 一定要先设置选中才可以
            bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')  # 一定要设置这个
            bpy.context.scene.objects[shape_file].select = False
            bpy.ops.rigidbody.object_add(type='ACTIVE')
            bpy.ops.object.modifier_add(type='COLLISION')
            # bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')    # 一定要设置这个
            # bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_MASS')    # 一定要设置这个
            bpy.context.scene.objects[shape_file].rigid_body.mass = 10.0    # 质量
            bpy.context.scene.objects[shape_file].rigid_body.use_margin = True    # 碰撞边距
            bpy.context.scene.objects[shape_file].rigid_body.collision_margin = 0    # 边距
            bpy.context.scene.objects[shape_file].rigid_body.linear_damping = 0.9    # 移动阻尼
            bpy.context.scene.objects[shape_file].rigid_body.angular_damping = 0.9    # 旋转阻尼

        ## performing simulation
        framesIter = cfg.getNumSimulationSteps()
        for i in range(1, framesIter):
            # Grab data from every frame
            bpy.context.scene.frame_set(i)

        removed_objs = list()
        # 查看所有没有被放起来的物体
        z = np.array([0, 0, -1])    # 世界坐标系中z的负半轴
        for obj in objectlist:
            if not bpy.data.objects[obj].hide:
                matrix_world = bpy.data.objects[obj].matrix_world
                y = np.array([matrix_world[0][1], matrix_world[1][1], matrix_world[2][1]])
                print("vector y:", y)
                angle = np.arccos(np.clip(np.dot(z, y), -1.0, 1.0))
                print('angle:', angle)

                if abs(angle) < 0.5:    # 如果物体的y和世界坐标系的z轴负半轴接近重合
                    bpy.data.objects[obj].hide = True    # # 隐藏物体
                    bpy.data.objects[obj].hide_render = True
                    removed_objs.append(obj)
                # bpy.context.scene.frame_set(0)
                # bpy.data.objects[obj].hide = True
                # bpy.data.objects[obj].hide_render = True
                # bpy.data.objects[obj].location[0] = 100.0

        # pick lighting
        light_range_x = cfg.getLightRangeX()
        light_range_y = cfg.getLightRangeY()
        light_range_z = cfg.getLightRangeZ()
        pLight.placePointLight(light_range_x, light_range_y, light_range_z)

        # rendering configuration
        for area in bpy.context.screen.areas:
            if area.type == 'VIEW_3D':
                area.spaces[0].region_3d.view_perspective = 'CAMERA'
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        space.viewport_shade = 'TEXTURED'

        bpy.context.scene.render.use_raytrace = False
        bpy.context.scene.render.use_shadows = False    # ?

        for i in range(0, cam.numViews):
            cam.placeCamera(i)
            output_img = "rendered_images/image_%05i.png" % num
            bpy.context.scene.render.filepath = os.path.join(g_repo_path, output_img)
            bpy.ops.render.render(write_still=True)

            if cfg.getLabelType() == 'pixel':
                for j in range(0, numObjectsInScene):
                    # make all items invisible
                    for item in bpy.data.materials:
                        item.use_transparency = True
                        item.transparency_method = 'MASK'
                        item.alpha = 0

                    index = selectedobj[j]
                    shape_file = objectlist[index]
                    if shape_file in removed_objs:
                        continue
                    bpy.data.objects[shape_file].material_slots[0].material.use_transparency = False
                    output_img = "rendered_images/debug/image_%05i_%02i.png" % (num, j)
                    bpy.context.scene.render.filepath = os.path.join(g_repo_path, output_img)
                    bpy.ops.render.render(write_still=True)

                # restore the transparency
                for item in bpy.data.materials:
                    item.use_transparency = False

            # 2-D bounding boxes
            output_bbox = "rendered_images/debug/raw_bbox_%05i.txt" % num
            for i in range(0, numObjectsInScene):
                index = selectedobj[i]    # 物体的类别索引
                shape_file = sceneobjectlist[index]    # 物体的文件名
                if shape_file in removed_objs:    # 如果这个物体已经被删除了,就跳过这个物体
                    continue
                print(index, shape_file)
                x, y, width, height = cam.write_bounds_2d(output_bbox, bpy.data.objects[shape_file], index)

            # save to temp.blend
            mainfile_path = os.path.join("rendered_images/debug/", "blend_%05d.blend" % num)
            bpy.ops.file.autopack_toggle()
            bpy.ops.wm.save_as_mainfile(filepath=mainfile_path)

            num = num + 1
            if num >= numImages:
                break
