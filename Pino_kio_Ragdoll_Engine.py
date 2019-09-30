
bl_info = {
    "name": "Pino_Kio Ragdoll Engine",
    "description":  "Adds a Ragdoll Rig to a Skeleton",
    "author": "Noizirom",
    "version": (0, 0, 3),
    "blender": (2, 80, 0),
    "location": "View3D > Add > Armature > Pino_Kio Ragdoll Engine",
    "warning": "", 
    "wiki_url": "https://github.com/Noizirom/Pino_Kio-Ragdoll-Engine",
    "tracker_url": "",
    "category": "Add Armature"
}


import bpy, numpy as np
from copy import deepcopy as dc
from mathutils import Matrix, Vector, Euler
from math import radians, degrees
from bpy_extras.object_utils import AddObjectHelper, object_data_add
from bpy.props import (StringProperty,
                       BoolProperty,
                       IntProperty,
                       FloatProperty,
                       PointerProperty
                       )
from bpy.types import (Panel,
                       Operator,
                       PropertyGroup,
                       )

# ------------------------------------------------------------------------
#    Initializations
# ------------------------------------------------------------------------


cap_co = np.array([[0.38268, 0.0, 1.92388], [0.2706, 0.2706, 1.92388], [0.0, 0.38268, 1.92388], [-0.2706, 0.2706, 1.92388], [-0.38268, 0.0, 1.92388], [-0.2706, -0.2706, 1.92388], [-0.0, -0.38268, 1.92388], [0.2706, -0.2706, 1.92388], [0.70711, 0.0, 1.70711], [0.5, 0.5, 1.70711], [0.0, 0.70711, 1.70711], [-0.5, 0.5, 1.70711], [-0.70711, 0.0, 1.70711], [-0.5, -0.5, 1.70711], [-0.0, -0.70711, 1.70711], [0.5, -0.5, 1.70711], [0.92388, 0.0, 1.38268], [0.65328, 0.65328, 1.38268], [0.0, 0.92388, 1.38268], [-0.65328, 0.65328, 1.38268], [-0.92388, 0.0, 1.38268], [-0.65328, -0.65328, 1.38268], [-0.0, -0.92388, 1.38268], [0.65328, -0.65328, 1.38268], [1.0, 0.0, 1.0], [0.70711, 0.70711, 1.0], [0.0, 1.0, 1.0], [-0.70711, 0.70711, 1.0], [-1.0, 0.0, 1.0], [-0.70711, -0.70711, 1.0], [-0.0, -1.0, 1.0], [0.70711, -0.70711, 1.0], [0.0, 0.0, 2.0], [0.38268, 0.0, -1.92388], [0.2706, 0.2706, -1.92388], [0.0, 0.38268, -1.92388], [-0.2706, 0.2706, -1.92388], [-0.38268, 0.0, -1.92388], [-0.2706, -0.2706, -1.92388], [-0.0, -0.38268, -1.92388], [0.2706, -0.2706, -1.92388], [0.70711, 0.0, -1.70711], [0.5, 0.5, -1.70711], [0.0, 0.70711, -1.70711], [-0.5, 0.5, -1.70711], [-0.70711, 0.0, -1.70711], [-0.5, -0.5, -1.70711], [-0.0, -0.70711, -1.70711], [0.5, -0.5, -1.70711], [0.92388, 0.0, -1.38268], [0.65328, 0.65328, -1.38268], [0.0, 0.92388, -1.38268], [-0.65328, 0.65328, -1.38268], [-0.92388, 0.0, -1.38268], [-0.65328, -0.65328, -1.38268], [-0.0, -0.92388, -1.38268], [0.65328, -0.65328, -1.38268], [1.0, 0.0, -1.0], [0.70711, 0.70711, -1.0], [0.0, 1.0, -1.0], [-0.70711, 0.70711, -1.0], [-1.0, 0.0, -1.0], [-0.70711, -0.70711, -1.0], [-0.0, -1.0, -1.0], [0.70711, -0.70711, -1.0], [0.0, 0.0, -2.0]])
cap_fa = [(24, 25, 17, 16), (58, 57, 49, 50), (25, 26, 18, 17), (59, 58, 50, 51), (26, 27, 19, 18), (60, 59, 51, 52), (27, 28, 20, 19), (61, 60, 52, 53), (28, 29, 21, 20), (62, 61, 53, 54), (29, 30, 22, 21), (63, 62, 54, 55), (30, 31, 23, 22), (64, 63, 55, 56), (31, 24, 16, 23), (57, 64, 56, 49), (16, 17, 9, 8), (50, 49, 41, 42), (17, 18, 10, 9), (51, 50, 42, 43), (18, 19, 11, 10), (52, 51, 43, 44), (19, 20, 12, 11), (53, 52, 44, 45), (20, 21, 13, 12), (54, 53, 45, 46), (21, 22, 14, 13), (55, 54, 46, 47), (22, 23, 15, 14), (56, 55, 47, 48), (23, 16, 8, 15), (49, 56, 48, 41), (8, 9, 1, 0), (42, 41, 33, 34), (9, 10, 2, 1), (43, 42, 34, 35), (10, 11, 3, 2), (44, 43, 35, 36), (11, 12, 4, 3), (45, 44, 36, 37), (12, 13, 5, 4), (46, 45, 37, 38), (13, 14, 6, 5), (47, 46, 38, 39), (14, 15, 7, 6), (48, 47, 39, 40), (15, 8, 0, 7), (41, 48, 40, 33), (25, 24, 57, 58), (26, 25, 58, 59), (27, 26, 59, 60), (28, 27, 60, 61), (29, 28, 61, 62), (30, 29, 62, 63), (31, 30, 63, 64), (24, 31, 64, 57), (0, 1, 32), (1, 2, 32), (2, 3, 32), (3, 4, 32), (4, 5, 32), (5, 6, 32), (6, 7, 32), (7, 0, 32), (34, 33, 65), (35, 34, 65), (36, 35, 65), (37, 36, 65), (38, 37, 65), (39, 38, 65), (40, 39, 65), (33, 40, 65)]

obj = bpy.data.objects
col_nme = "Ragdoll"
ext = 'UCX_'
tgt = '_target'
coll = bpy.data.collections

# ------------------------------------------------------------------------
#    Functions
# ------------------------------------------------------------------------


###############################################################################################################################
#LIST OPS

def flat_list(List):
    return [i[j] for i in List for j in range(len(i))]

def list_inclusion(List, Ref):
    List, Ref = list(List), list(Ref)
    return [[i for i in List if j in i] for j in Ref]

def list_exclusion(List, Ref):
    List, Ref = list(List), list(Ref)
    return [i for i in List if not any(j in i for j in Ref)]

def name_list(Ob):
    return [i.name for i in Ob]

def name_rm_ext(Name, ext):
    return Name.replace(ext, '')

def list_split_sides(List):
    lt = [i for i in List if '_l' in i or '_L' in i]
    rt = [i for i in List if '_r' in i or '_R' in i]   
    return lt, rt

def bone_dict(bones):
    '''
    {name: [ #0 index, #1 head, #2 head_local, #3 head_radius, 
    #4 center, #5 tail, #6 tail_local, #7 tail_radius, #8 length, 
    #9 matrix, #10 matrix_local, #11 x_axis, #12 y_axis, #13 z_axis,
    #14 vector, #15 parent ]}
    '''
    bd = {bone.name: 
    [idx, bone.head, bone.head_local, bone.head_radius, 
    bone.center, bone.tail, bone.tail_local, bone.tail_radius, bone.length, 
    bone.matrix, bone.matrix_local ,bone.x_axis, bone.y_axis, bone.z_axis,
    bone.vector, bone.parent]
    for idx, bone in enumerate(bones)}
    return bd

    


###############################################################################################################################
#**************************************REMOVE PREVIOUS ADDED LIMIT CONSTRAINTS

def remove_bone_constraints(constraint, pb):
    for bone in pb:
        rbc = [r for r in bone.constraints if r.type == constraint]
        for r in rbc:
            bone.constraints.remove(r)


###############################################################################################################################
#************************************************************ADD LIMIT SCALE

def limit_bone_scale(pb):
    for bone in pb:
        bc = bone.constraints.new(type='LIMIT_SCALE')
        bc.owner_space = 'LOCAL'
        bc.use_min_x = True
        bc.use_max_x = True
        bc.min_x = 1.0
        bc.max_x = 1.0
        bc.use_min_y = True
        bc.use_max_y = True
        bc.min_y = 1.0
        bc.max_y = 1.0
        bc.use_min_z = True
        bc.use_max_z = True
        bc.min_z = 1.0
        bc.max_z = 1.0

#************************************************************ADD LIMIT ROTATION

def limit_bone_rotation(Dict, pb):
    for bone in pb:
        if bone.name in Dict:
            bc = bone.constraints.new(type='LIMIT_ROTATION')
            bc.owner_space = 'LOCAL'
            bc.use_limit_x = True
            bc.use_limit_y = True
            bc.use_limit_z = True
            bc.min_x = radians(Dict[bone.name][0])
            bc.max_x = radians(Dict[bone.name][1])
            bc.min_y = radians(Dict[bone.name][2])
            bc.max_y = radians(Dict[bone.name][3])
            bc.min_z = radians(Dict[bone.name][4])
            bc.max_z = radians(Dict[bone.name][5])

#************************************************************ADD COPY TRANSFORMATION

def copy_empty_transform(pb, bl, infl):
    for bo in pb:
        if bo.name in bl:
            bc = bo.constraints.new(type='COPY_TRANSFORMS')
            bc.target = obj[bo.name + tgt]
            bc.influence = infl
            
def copy_bone_transform(Dict):
    for b in Dict:
        if not Dict[b][6]:
            pass
        else:
            bc = obj[ext + b].constraints.new(type='COPY_TRANSFORMS')
            bc.target = obj[arm.name]
            bc.subtarget = b

#************************************************************ADD LIMIT DISTANCE

def limit_target_distance(Dict):
    for emp in Dict:
        if not Dict[emp][6]:
            pass
        else:
            ec = obj[emp + tgt].constraints.new(type='LIMIT_DISTANCE')
            ec.target = obj[ext + Dict[emp][6]]
            ec.limit_mode = 'LIMITDIST_ONSURFACE'
            ec.use_transform_limit = True
            ec.target_space = 'LOCAL'
            ec.owner_space = 'LOCAL'
        

###############################################################################################################################
#ADD RIGIDBODY CONSTRAINTS
try:
    bpy.ops.rigidbody.world_remove()
except:
    pass
bpy.ops.rigidbody.world_add()


def add_rbc(part, Dict):
    if not Dict[part][6]:
        pass
    else:
        #rg_list
        rig = part + tgt
        p1 = ext + Dict[part][6]
        p2 = ext + part
        #rd_dict[part]
        xl = Dict[part][0]
        xu = Dict[part][1]
        yl = Dict[part][2]
        yu = Dict[part][3]
        zl = Dict[part][4]
        zu = Dict[part][5]
        bpy.ops.object.select_all(action='DESELECT')
        obj[rig].select_set(state=True)
        bpy.context.view_layer.objects.active = obj[rig]
        bpy.ops.rigidbody.constraint_add(type = 'GENERIC')
        #
        rbc = bpy.context.object.rigid_body_constraint
        rbc.object1 = obj[p1]
        rbc.object2 = obj[p2]
        ###use limits###
        #angle
        rbc.use_limit_ang_x = True
        rbc.use_limit_ang_y = True
        rbc.use_limit_ang_z = True
        rbc.use_limit_lin_x = True
        rbc.use_limit_lin_y = True
        rbc.use_limit_lin_z = True
        #Linear
        rbc.limit_lin_x_lower = 0
        rbc.limit_lin_x_upper = 0
        rbc.limit_lin_y_lower = 0
        rbc.limit_lin_y_upper = 0
        rbc.limit_lin_z_lower = 0
        rbc.limit_lin_z_upper = 0
        #Rotation Constraints
        rbc.limit_ang_x_lower = radians(xl)
        rbc.limit_ang_x_upper = radians(xu)
        rbc.limit_ang_y_lower = radians(yl)
        rbc.limit_ang_y_upper = radians(yu)
        rbc.limit_ang_z_lower = radians(zl)
        rbc.limit_ang_z_upper = radians(zu)
        obj[rig].select_set(state=False)
        obj[p2].select_set(state=True)
        print(rig+"<<<finished>>>")

def rigid_enable(part):
    obj[ext + part].rigid_body.enabled = True

def rigid_disable(part):
    obj[ext + part].rigid_body.enabled = False


###############################################################################################################################
#OBJECT CREATION

def rd_capsule(length, radius):
    sc = np.eye(3)
    sc[0][0] = radius
    sc[1][1] = radius
    sc[2][2] = length/4
    coo = np.array([[i[0], i[1], i[2] + 2] for i in cap_co])
    nc = coo @ sc
    return nc

def obj_mesh(co, faces):
    mesh = bpy.data.meshes.new("Obj")
    mesh.from_pydata(co, [], faces)
    mesh.validate()
    mesh.update(calc_edges = True)
    Object = obj.new("Obj", mesh)
    Object.data = mesh
    coll['Collection'].objects.link(Object)
    bpy.context.view_layer.objects.active = Object
    Object.select_set(True)

def obj_new(Name, co, faces):
    obj_mesh(co, faces)
    obj["Obj"].name = Name
    bpy.data.meshes[obj[Name].data.name].name = Name

def add_capsule(Name, length, radius):
    cor = rd_capsule(length, radius)
    obj_new(Name, cor, cap_fa)
    try:
        bpy.ops.rigidbody.objects_add(type='ACTIVE')        
    except:
        pass



def rotation_matrix(xrot, yrot, zrot):
    rot_mat = np.array(
                        [ [np.cos(xrot)*np.cos(yrot), -np.sin(xrot)*np.cos(zrot) + np.cos(xrot)*np.sin(yrot)*np.sin(zrot), np.sin(xrot)*np.sin(zrot) + np.cos(xrot)*np.sin(yrot)*np.cos(zrot)],
                        [ np.sin(xrot)*np.cos(yrot), np.cos(xrot)*np.cos(zrot) + np.sin(xrot)*np.sin(yrot)*np.sin(zrot), -np.cos(xrot)*np.sin(zrot) + np.sin(xrot)*np.sin(yrot)*np.cos(zrot)],
                        [-np.sin(yrot), np.cos(yrot)*np.sin(zrot), np.cos(yrot)*np.cos(zrot)] ]
                        )
    return rot_mat

def apply_mod(Ref):
    act = bpy.context.view_layer.objects.active
    for o in bpy.context.view_layer.objects:
        for m in o.modifiers:
            if Ref in m.name:
                bpy.context.view_layer.objects.active = o
                bpy.ops.object.modifier_apply(modifier=m.name)
    bpy.context.view_layer.objects.active = act

def obj_del(List):
    bpy.ops.object.select_all(action='DESELECT')
    for o in List:
        obj[o].select_set(state=True)
        bpy.ops.object.delete()

def vec_dist(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    return np.sqrt(np.sum((v1 - v2)**2))

###############################################################################################################################
#RAGDOLL CREATION

def torso_obj(mat, length, radius):
    n = ext + 'torso'
    add_capsule(n, length*2.25, radius)
    obj[n].matrix_world = mat @ Matrix(rotation_matrix(0,0,radians(-90))).to_4x4()
    obj[n].display_type = 'WIRE'


def ragdoll_obj(part, ext, mat, length, radius):
    p = ext + part
    add_capsule(p, length, radius) # Dict[part][8]*.9, Dict[part][7])
    obj[p].matrix_world = mat @ Matrix(rotation_matrix(0,0,radians(-90))).to_4x4()
    obj[p].display_type = 'WIRE'


def add_emp(pbn, Dict):
    tmp_final = Dict[pbn][10] @ Matrix(rotation_matrix(0,0,radians(-90))).to_4x4()
    matrix_final = tmp_final @ Matrix(rotation_matrix(0,0,radians(90))).to_4x4()
    obj_empty = obj.new(pbn + tgt, None)
    coll['Collection'].objects.link(obj_empty)
    #draw size
    obj_empty.empty_display_size = 0.1
    obj_empty.matrix_world = matrix_final


def add_parent(rd):
    try:
        bpy.ops.object.select_all(action='DESELECT')
        a = obj[ext + rd]
        b = obj[rd + tgt]
        a.select_set(state=True)
        b.select_set(state=True)
        bpy.context.view_layer.objects.active = a
        bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)
    except:
        pass

def add_parent_torso(part):
    try:
        add_parent("torso")
        bpy.ops.object.select_all(action='DESELECT')
        par = obj[ext + "torso"]
        t = flat_list(part)
        for i in t:
            obj[i + tgt].select_set(state=True)
        par.select_set(state=True)
        bpy.context.view_layer.objects.active = par
        bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)
    except:
        pass
    
def add_collection(List, List2):
    for o in List:
        obj[ext + o].select_set(state=True)
    for e in List2:
        obj[e + tgt].select_set(state=True)
    obj["torso" + tgt].select_set(state=True)
    try:
        if coll[col_nme]:
            coll.remove(coll[col_nme])
    except:
        pass
    bpy.ops.object.move_to_collection(collection_index=0, is_new=True, new_collection_name=col_nme)


def Part_names():
    pb = bpy.context.object.pose.bones
    xtrb = ['twist', 'root', 'muscle', 'breast', 'IK_control_', 'struct_', 'rot_helper']
    hand =['hand', 'thumb', 'index', 'middle', 'ring', 'pinky']
    hdb = ['head']
    pel = ['pelvis']
    torso = ['neck', 'spine', 'clavicle']
    legs = ['thigh', 'calf']
    foot = ['foot', 'toe']
    rl = xtrb + hand + foot
    b_list = name_list(pb)
    bl = list_exclusion(b_list, rl)
    b_xtrb = list_inclusion(b_list, xtrb)
    b_head = list_inclusion(bl, hdb)
    b_torso = list_inclusion(bl, torso)
    b_arms = [i for i in bl if 'arm' in i]
    b_hand = list_inclusion(b_list, hand)
    b_leg = list_inclusion(bl, legs)
    b_foot = list_inclusion(b_list, foot)
    #center mass
    head = b_head[0][0]
    neck = b_torso[0][0]
    clavicle_l = b_torso[2][0]
    clavicle_r = b_torso[2][1]
    #spine
    chest = b_torso[1][2]
    abdomen = b_torso[1][1]
    hip = b_torso[1][0]
    #pelvis
    pelvis = bl[0]
    #legs
    thigh_l = b_leg[0][1]
    calf_l = b_leg[1][1]
    foot_l = b_foot[0][1]
    thigh_r = b_leg[0][0]
    calf_r = b_leg[1][0]
    foot_r = b_foot[0][0]
    #arms
    upperarm_l = b_arms[0]
    lowerarm_l = b_arms[1]
    upperarm_r = b_arms[2]
    lowerarm_r = b_arms[3]
    #hands
    hand_l = b_hand[0][0]
    hand_r = b_hand[0][1]
    #fingers
    thumb_l, thumb_r = list_split_sides(b_hand[1])
    index_l, index_r = list_split_sides(b_hand[2])
    middle_l, middle_r = list_split_sides(b_hand[3])
    ring_l, ring_r = list_split_sides(b_hand[4])
    pinky_l, pinky_r = list_split_sides(b_hand[5])
    #ragdoll list
    hands = b_hand[0]
    feet = b_foot[0]
    limbs = b_arms + b_leg[0] + b_leg[1]
    rdl = limbs + [head, pelvis]
    emp_list = bl + hands + feet
    return head, neck, clavicle_l, clavicle_r, chest, abdomen, hip, pelvis, thigh_l, calf_l, foot_l, thigh_r, calf_r, foot_r, upperarm_l, lowerarm_l, upperarm_r, lowerarm_r, hand_l, hand_r, thumb_l, thumb_r, index_l, index_r, middle_l, middle_r, ring_l, ring_r, pinky_l, pinky_r, hands, feet, limbs, b_torso, rdl, emp_list, bl


###############################################################################################################################
#MAIN

def Ragdoll(self, context):
    head, neck, clavicle_l, clavicle_r, chest, abdomen, hip, pelvis, thigh_l, calf_l, foot_l, thigh_r, calf_r, foot_r, upperarm_l, lowerarm_l, upperarm_r, lowerarm_r, hand_l, hand_r, thumb_l, thumb_r, index_l, index_r, middle_l, middle_r, ring_l, ring_r, pinky_l, pinky_r, hands, feet, limbs, b_torso, rdl, emp_list, bl = Part_names()
    infl = bpy.context.scene.ragdoll_rig.rd_influence
    arm = bpy.context.object
    bones = bpy.data.armatures[0].bones
    pb = bpy.context.object.pose.bones
    amw = arm.matrix_world.copy()
    body_weight = 79 #150 lbs
    p_mass = 1
    bd = bone_dict(bones)
    body_height = bd[head][6][2]
    p_orig_ht = bd[pelvis][2][2]
    rd_dict = {
            head: [-22, 37, -45, 45, -30, 30, 'torso', .08],
            neck: [-22, 37, -45, 45, -30, 30, '', ''],
            clavicle_l: [-30, 30, 0, 0, -30, 10, '', ''],
            clavicle_r: [-30, 30, 0, 0, -10, 30, '', ''],
            'torso': [-1, 2, -2, 2, -1, 1, pelvis, .5],
            pelvis: [0, 0, 0, 0, 0, 0, '', ''],
            hip: [-45, 68, -45, 45, -30, 30,'', ''],
            abdomen: [-45, 68, -45, 45, -30, 30,'', ''],
            chest: [-45, 22, -45, 45, -30, 30, '', ''],
            upperarm_l: [-58, 95, -30, 15, -60, 105, 'torso', .03],
            upperarm_r: [-58, 95, -30, 15, -60, 105, 'torso', .03],
            lowerarm_l: [-146, 0, -15, 0, 0, 0, upperarm_l, .014],
            lowerarm_r: [-146, 0, 0, 15, 0, 0, upperarm_r, .014],
            hand_l: [-45, 45, -90, 86, -25, 36, lowerarm_l, .006],
            hand_r: [-45, 45, -86, 90, -36, 25, lowerarm_r, .006],
            thigh_l: [-90, 45, -15, 15, -22, 17, pelvis, .1],
            thigh_r: [-90, 45, -15, 15, -22, 17, pelvis, .1],
            calf_l: [0, 150, 0, 0, 0, 0, thigh_l, .05],
            calf_r: [0, 150, 0, 0, 0, 0, thigh_r, .05],
            foot_l: [-44, 45, -26, 26, -15, 74, calf_l, .01],
            foot_r: [-45, 44, -26, 26, -74, 15, calf_r, .01]
            }
    limit_bone_scale(pb)
    limit_bone_rotation(rd_dict, pb)
    for p in limbs:
        ragdoll_obj(p, ext, bd[p][10], bd[p][8]*.9, bd[p][7])
    ragdoll_obj(pelvis, ext, bd[pelvis][10], bd[pelvis][8], bd[clavicle_l][8]*.95)
    ragdoll_obj(head, ext, bd[head][10], bd[head][8], bd[head][8]/2)
    for p in hands:
        ragdoll_obj(p, ext, bd[p][10], bd[p][3], bd[p][8])
    for p in feet:
        ragdoll_obj(p, ext, bd[p][10], bd[p][3]*2, bd[p][8]/4)    
    torso_length = vec_dist(bd[hip][1], bd[neck][5])
    torso_obj(bd[hip][10], torso_length, bd[clavicle_l][8])
    torso_dist = (vec_dist(obj[ext + "torso"].location.y, obj[ext + head].location.y))/2
    obj[ext + "torso"].location.y = obj[ext + "torso"].location.y - torso_dist
    add_emp(hip, bd)
    obj[hip + tgt].name = "torso" + tgt
    for b in emp_list:
        add_emp(b, bd)
    for bn in rdl:
        add_parent(bn)
    for bn in hands:
        add_parent(bn)
    for bn in feet:
        add_parent(bn)        
    for p in rd_dict:
        add_rbc(p, rd_dict)
    for c in rd_dict:
        if not rd_dict[c][7]:
            pass
        else:
            obj[ext + c].rigid_body.mass = body_weight * rd_dict[c][7]
            obj[ext + c].rigid_body.collision_shape = 'CAPSULE'
            obj[ext + pelvis].rigid_body.mass = p_mass
    obj[ext + pelvis].rigid_body.collision_shape = 'CAPSULE'
    bpy.ops.object.select_all(action='DESELECT')
    add_parent_torso(b_torso)
    add_collection((rdl + hands + feet), emp_list)
    copy_empty_transform(pb, bl, infl)
    bpy.ops.object.select_all(action='DESELECT')


# ------------------------------------------------------------------------

# This allows you to right click on a button and link to documentation
def pino_kio_ragdoll_manual_map():
    url_manual_prefix = "https://docs.blender.org/manual/en/latest/"
    url_manual_mapping = (
        ("bpy.ops.mesh.add_object", "scene_layout/object/types.html"),
    )
    return url_manual_prefix, url_manual_mapping

# ------------------------------------------------------------------------
#    Scene Properties
# ------------------------------------------------------------------------

class RR_Properties(PropertyGroup):

    rd_influence: FloatProperty(
        name="Ragdoll Influence",
        description="Ragdoll Influence",
        default = 1.0,
        min = 0.0,
        max = 1.0,
        )
'''
    add_origin_bool: BoolProperty(
        name="Origin at Centroid",
        description="Place origin at center of object",
        default = False
        )

    set_smooth_bool: BoolProperty(
        name = "Set Smooth",
        description = "New Object Set Smooth",
        default = False
        )

    obj_name: StringProperty(
        name="name",
        description="Enter the Object Name",
        default="Object",
        maxlen=1024,
        )
'''

# ------------------------------------------------------------------------
#    Operators
# ------------------------------------------------------------------------

class OBJECT_OT_pino_kio_ragdoll(Operator):
    """Add a Ragdoll Rig to an Armature"""
    bl_idname = "object.pino_kio_ragdoll"
    bl_label = "Pino_Kio Ragdoll"
    bl_options = {'REGISTER', 'UNDO'}
    
    
    
    def execute(self, context):
        Ragdoll(self, context)
        return {'FINISHED'}

# ------------------------------------------------------------------------
#    Panel in Object Mode
# ------------------------------------------------------------------------

class OBJECT_PT_pino_kio_ragdoll(Panel):
    bl_idname = "object.ragdoll_panel"
    bl_label = "Pino_Kio Ragdoll"
    bl_space_type = "VIEW_3D"   
    bl_region_type = "UI"
    bl_category = "Ragdoll"
    
    @classmethod
    def poll(self,context):
        return context.mode in {'OBJECT', 'EDIT_MESH'}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        rd_rig = scene.ragdoll_rig

        layout.prop(rd_rig, "rd_influence")
        '''
        layout.prop(rd_rig, "add_origin_bool")
        layout.prop(rd_rig, "set_smooth_bool")
        layout.prop(rd_rig, "obj_name")
        '''
        layout.operator("object.pino_kio_ragdoll")
        layout.separator()



# ------------------------------------------------------------------------
#    Registration
# ------------------------------------------------------------------------

classes = (
    RR_Properties,
    OBJECT_OT_pino_kio_ragdoll,
    OBJECT_PT_pino_kio_ragdoll,
)



def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)

    bpy.types.Scene.ragdoll_rig = PointerProperty(type=RR_Properties)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)
    del bpy.types.Scene.ragdoll_rig


if __name__ == "__main__":
    register()



