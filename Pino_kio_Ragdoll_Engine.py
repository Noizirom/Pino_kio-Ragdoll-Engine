bl_info = {
    "name": "Pino_Kio Ragdoll Engine",
    "description":  "Adds a Ragdoll Rig to a Skeleton",
    "author": "Noizirom",
    "version": (0, 0, 5),
    "blender": (2, 80, 1),
    "location": "View3D > Add > Armature > Pino_Kio Ragdoll Engine",
    "warning": "", 
    "wiki_url": "https://github.com/Noizirom/Pino_Kio-Ragdoll-Engine",
    "tracker_url": "",
    "category": "Add Object"
}


import bpy
import numpy as np
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

###############################################################################################################################
### CONTAINERS ###
###############################################################################################################################


default_names = ["head",   #0
            "neck",        #1
            "clavicle_L",  #2
            "clavicle_R",  #3
            "upperarm_L",  #4
            "upperarm_R",  #5
            "lowerarm_L",  #6
            "lowerarm_R",  #7
            "hand_L",      #8
            "hand_R",      #9
            "thigh_L",     #10
            "thigh_R",     #11
            "calf_L",      #12
            "calf_R",      #13
            "foot_L",      #14
            "foot_R",      #15
            "spine01",     #16
            "spine02",     #17
            "spine03",     #18
            "pelvis",      #19
            "breast_L",    #20
            "breast_R",    #21
]

def body_sections(container):
    arms = ["{}".format(container[4]), "{}".format(container[6]), "{}".format(container[5]), "{}".format(container[7])]
    legs = ["{}".format(container[10]), "{}".format(container[12]), "{}".format(container[11]), "{}".format(container[13])]
    spine = ["{}".format(container[16]), "{}".format(container[17]), "{}".format(container[18])]
    return arms, legs, spine

def default_fingers():
    '''
    Returns 4 items
    #0 Flat list of left hand fingers
    #1 Flat list of right hand fingers
    #2 Nested list of left hand fingers from thumb to pinky
    #3 Nested list of right hand fingers from thumb to pinky
    '''
    thumb_l = ["thumb01_L", "thumb02_L", "thumb03_L"]
    index_l = ["index00_L", "index01_L", "index02_L", "index03_L"]
    middle_l = ["middle00_L", "middle01_L", "middle02_L", "middle03_L"]
    ring_l = ["ring00_L", "ring01_L", "ring02_L", "ring03_L"]
    pinky_l = ["pinky00_L", "pinky01_L", "pinky02_L", "pinky03_L"]
    thumb_r = ["thumb01_R", "thumb02_R", "thumb03_R"]
    index_r = ["index00_R", "index01_R", "index02_R", "index03_R"]
    middle_r = ["middle00_R", "middle01_R", "middle02_R", "middle03_R"]
    ring_r = ["ring00_R", "ring01_R", "ring02_R", "ring03_R"]
    pinky_r = ["pinky00_R", "pinky01_R", "pinky02_R", "pinky03_R"]
    lh = [thumb_l, index_l, middle_l, ring_l, pinky_l]
    left_hand =[]
    [left_hand.extend(i) for i in lh]
    rh = [thumb_r, index_r, middle_r, ring_r, pinky_r]
    right_hand =[]
    [right_hand.extend(i) for i in lh]
    return left_hand, right_hand, lh, rh

#Dictionary for fingers
def finger_dict(lh_list, rh_list, container):
    fd = {}
    for item in range(len(lh_list)):
        for part in lh_list[item]:
            if part == lh_list[item][0]:
                target = container[8] #hand_L
                fd.update({part: [0,0,0,0,-5,5, target, 0.01]})
            else:
                idx = lh_list[item].index(part)- 1
                target = lh_list[item][idx]
                fd.update({part: [0,90,0,0,-5,5, target, 0.01]})
    for item in range(len(rh_list)):
        for part in rh_list[item]:
            if part == rh_list[item][0]:
                target = container[9] #hand_R
                fd.update({part: [0,0,0,0,-5,5, target, 0.01]})
            else:
                idx = rh_list[item].index(part)- 1
                target = rh_list[item][idx]
                fd.update({part: [0,90,0,0,-5,5, target, 0.01]})
    return fd

def ragdoll_dict(part_names):
    rd = {
        part_names[0]: [-22, 37, -45, 45, -30, 30, part_names[1], .06],             #head
        part_names[1]: [-22, 37, -45, 45, -30, 30, part_names[18], .02],            #neck
        part_names[2]: [-30, 30, 0, 0, -30, 10, part_names[1], .05],                #clavicle_L
        part_names[3]: [-30, 30, 0, 0, -10, 30, part_names[1], .05],                #clavicle_R
        part_names[20]: [-10, 10, 0, 0, 0, 0, part_names[18], .05],                 #breast_L
        part_names[21]: [-10, 10, 0, 0, 0, 0, part_names[18], .05],                 #breast_R
        part_names[19]: [-22, 45, -45, 45, -15, 15, '', .1],                        #pelvis
        part_names[16]: [-45, 68, -45, 45, -30, 30, part_names[19], .1],            #spine_01
        part_names[17]: [-45, 22, -45, 45, -30, 30, part_names[16], .2],            #spine_02
        part_names[18]: [-45, 22, -45, 45, -30, 30, part_names[17], .1],            #spine_03
        part_names[4]: [-58, 95, -30, 15, -60, 105, part_names[2], .03],            #upperarm_L
        part_names[5]: [-58, 95, -30, 15, -60, 105, part_names[3], .03],            #upperarm_R
        part_names[6]: [-146, 0, -15, 0, 0, 0, part_names[4], .014],                #lowerarm_L
        part_names[7]: [-146, 0, -15, 0, 0, 0, part_names[5], .014],                #lowerarm_R
        part_names[8]: [-30, 30, -5, 5, -25, 36, part_names[6], .006],              #hand_L
        part_names[9]: [-30, 30, -5, 5, -36, 25, part_names[7], .006],              #hand_R
        part_names[10]: [-90, 45, -15, 15, -22, 17, part_names[19], .1],            #thigh_L
        part_names[11]: [-90, 45, -15, 15, -22, 17, part_names[19], .1],            #thigh_R
        part_names[12]: [0, 150, 0, 0, 0, 0, part_names[10], .05],                  #calf_L
        part_names[13]: [0, 150, 0, 0, 0, 0, part_names[11], .05],                  #calf_R
        part_names[14]: [-44, 45, -26, 26, -15, 74, part_names[12], .01],           #foot_L
        part_names[15]: [-45, 44, -26, 26, -74, 15, part_names[13], .01],           #foot_R
        }
    return rd

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

def posebone_dict(armature_ob):
    '''
    {name: [ #0 index, #1 head, #2 center, #3 tail,
    #4 length, #5 vector, #6 matrix, #7 matrix.translation #8 matrix.to_euler #9 parent]}
    '''
    pb = armature_ob.pose.bones
    pbd = {bone.name: 
    [idx, bone.head, bone.center, bone.tail, bone.length, bone.vector, bone.matrix, bone.matrix.translation, bone.matrix.to_euler, bone.parent]
    for idx, bone in enumerate(pb)}
    return pbd

def rd_parents(container, extension):
    bp = [
        ["{}".format(container[0]), "{}".format(container[1])],
        ["{}".format(container[1]), "{}".format(container[18])],
        ["{}".format(container[2]), "{}".format(container[18])],
        ["{}".format(container[3]), "{}".format(container[18])],
        ["{}".format(container[4]), "{}".format(container[2])],
        ["{}".format(container[5]), "{}".format(container[3])],
        ["{}".format(container[6]), "{}".format(container[4])],
        ["{}".format(container[7]), "{}".format(container[5])],
        ["{}".format(container[8]), "{}".format(container[6])],
        ["{}".format(container[9]), "{}".format(container[7])],
        ["{}".format(container[10]), "{}".format(container[19])],
        ["{}".format(container[11]), "{}".format(container[19])],
        ["{}".format(container[12]), "{}".format(container[10])],
        ["{}".format(container[13]), "{}".format(container[11])],
        ["{}".format(container[14]), "{}".format(container[12])],
        ["{}".format(container[15]), "{}".format(container[13])],
        ["{}".format(container[16]), "{}".format(container[19])],
        ["{}".format(container[17]), "{}".format(container[16])],
        ["{}".format(container[18]), "{}".format(container[17])],
        ["{}".format(container[20]), "{}".format(container[18])],
        ["{}".format(container[21]), "{}".format(container[18])],
    ]
    rdp = [
        ["{}_{}".format(extension, container[0]), "{}_{}".format(extension, container[1])],
        ["{}_{}".format(extension, container[1]), "{}_{}".format(extension, container[18])],
        ["{}_{}".format(extension, container[2]), "{}_{}".format(extension, container[18])],
        ["{}_{}".format(extension, container[3]), "{}_{}".format(extension, container[18])],
        ["{}_{}".format(extension, container[16]), "{}_{}".format(extension, container[19])],
        ["{}_{}".format(extension, container[17]), "{}_{}".format(extension, container[16])],
        ["{}_{}".format(extension, container[18]), "{}_{}".format(extension, container[17])],
        ["{}_{}".format(extension, container[20]), "{}_{}".format(extension, container[18])],
        ["{}_{}".format(extension, container[21]), "{}_{}".format(extension, container[18])],
    ]
    return bp, rdp

def get_collection(Name):
    '''get collection by name'''
    coll = bpy.data.collections
    nc = coll.get(Name)
    return nc

def new_collection(Name):
    '''create collection by name'''
    coll = bpy.data.collections
    nc = coll.new(Name)
    bpy.context.scene.collection.children.link(nc)


#df = default_fingers()

part_names = default_names
#part_names = (i for i in default_names)


###############################################################################################################################
### CLASSES ###
###############################################################################################################################


class RagdollPart:
    '''
    Create a Ragdoll Object 
    name is the name of the Ragdoll Part
    head is the coordinate of the origin og the Ragdoll Part
    tail is the tip of the Ragdoll Part
    up_vec is the vector of the up direction
    example:
    Z up would be a vector of (0,0,1)
    Y up would be a vector of (0,1,0)
    and so on ...

    '''
    def __init__(self, name, head, tail, up_vec, radius, extension, collection):
        self.name = name
        self.head = head
        self.tail = tail
        self.up_vec = np.array(up_vec, float)
        self.vec = tail - head
        self.unit_vec = self.vec / np.linalg.norm(self.vec)
        self.center = self.head + (self.vec / 2)
        self.magnitude = np.sqrt(np.sum((self.vec)**2))
        self.length = abs(self.magnitude)
        self.angle = np.arccos(np.dot(self.vec, self.up_vec))
        self.rotation = rot_mat_arb(self.vec, self.up_vec)
        self.matrix = matrix(self.rotation, self.head)
        self.matrix_inv = np.linalg.inv(self.matrix)
        self.radius = radius
        self.extension = extension
        self.ragdoll_name = self.extension + "_" + self.name
        self.RDjoint_name = "RDjoint_" + self.name
        self.collection = collection
    
    def ragdollpart(self, loc, rot):
        '''Creates Ragdoll Object'''
        obj = bpy.data.objects
        add_rd_capsule(self.ragdoll_name, self.length, self.radius, self.collection)
        rd = obj[self.ragdoll_name]
        rd.display_type = 'WIRE'
        vt = rd.data.vertices
        idx = len(vt) - 1
        add_empty(self.RDjoint_name, self.collection)
        add_parent(self.ragdoll_name, [self.RDjoint_name])
        jt = obj[self.RDjoint_name]
        jt.hide_render = True
        rd.hide_render = True
        
    def shrink_rd(self, target):
        '''add shrinkwrap modifier and apply'''
        obj = bpy.data.objects
        add_shrinkwrap(obj[self.ragdoll_name], target)
        apply_mod('shrink')

    def get_vec_angle(self, other):
        '''other is the vector measured to'''
        return rot_mat_arb(np.array(other) ,self.vec)

    def get_dist_vec(self, other):
        '''
        other is the point measured to
        returns the vector and distance for
        the head to point
        the center to point
        the tail to point
        '''
        h = self.head
        c = self.center
        t = self.tail
        d = np.array(other)
        hv = d - h
        cv = d - c
        tv = d - t
        return [[hv, abs(np.sqrt(np.sum((hv)**2)))], [cv, abs(np.sqrt(np.sum((cv)**2)))], [tv, abs(np.sqrt(np.sum((tv)**2)))] ]


class Ragdoll(RagdollPart):
    '''

    '''
    
    def __init__(self, armature, up_vec, extension, collection):
        self.armature = armature
        self.up_vec = up_vec
        self.extension = extension
        self.collection = collection
 
    def ragdoll(self, container):
        '''
        Creates the ragdoll
        adds physics
        adds rigid body constraints
        '''
        obj = bpy.data.objects
        arm = self.armature
        pb = arm.pose.bones
        bd = bone_dict(bpy.data.armatures[0].bones)
        new_collection(self.collection)
        bpy.ops.rigidbody.world_add()
        new_ragdoll(self.armature, self.up_vec, self.extension, self.collection, container)
        rd = ragdoll_dict(container)
        add_rigid_body(self.collection)
        for i in container:
            add_rbc(i, rd, self.extension)
        copy_empty_transform(pb, container, 1.0)
        bp, rdp = rd_parents(container, self.extension)
        for r in rdp:
            adoption(r[1], r[0], "OBJECT", None)



###############################################################################################################################
### DEFINITIONS ###
###############################################################################################################################

###############################################################################################################################
# MATH

def rot_mat_arb(a, b):
    '''rotation matrix between two vectors'''
    a = np.array(a)
    b = np.array(b)
    V = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]])
    c = np.sqrt( (b[0] - a[0])**2 + (b[1] - a[1])**2 + (b[2] - a[2])**2 )
    return [V[0]/ c, V[1]/ c, V[2]/ c]

def rotation_matrix(xrot, yrot, zrot):
    '''rotation matrix euler xyz rotations'''
    rot_mat = np.array(
                    [ [np.cos(-zrot)*np.cos(yrot), -np.sin(-zrot)*np.cos(xrot) + np.cos(-zrot)*np.sin(yrot)*np.sin(xrot), np.sin(-zrot)*np.sin(xrot) + np.cos(-zrot)*np.sin(yrot)*np.cos(xrot)],
                    [ np.sin(-zrot)*np.cos(yrot), np.cos(-zrot)*np.cos(xrot) + np.sin(-zrot)*np.sin(yrot)*np.sin(xrot), -np.cos(-zrot)*np.sin(xrot) + np.sin(-zrot)*np.sin(yrot)*np.cos(xrot)],
                    [-np.sin(yrot), np.cos(yrot)*np.sin(xrot), np.cos(yrot)*np.cos(xrot)] ]
                    )
    return rot_mat

def matrix(rot, loc):
    '''numpy 4x4 matrix from rotation and loation'''
    rm = rotation_matrix(rot[0], rot[1], rot[2])
    loc = Vector(loc)
    m = Matrix(rm).to_4x4()
    m[0][3] = loc[0]
    m[1][3] = loc[1]
    m[2][3] = loc[2]
    return np.array(m)

def get_distance(a, b):
    '''get the distance between two 3D points'''
    a = np.array(a)
    b = np.array(b)
    return abs(np.sqrt(np.sum((b-a)**2)))

def get_angle(self, other):
    '''get the angle between two vectors'''
    v = unit_vec(self)
    o = np.array(other)
    o = o / np.linalg.norm(o)
    return np.arccos(np.clip(np.dot(v, o), -1.0, 1.0))

###############################################################################################################################
#OBJECT CREATION

#creates new mesh
def obj_mesh(co, faces, collection):
    cur = bpy.context.object
    mesh = bpy.data.meshes.new("Obj")
    mesh.from_pydata(co, [], faces)
    mesh.validate()
    mesh.update(calc_edges = True)
    Object = bpy.data.objects.new("Obj", mesh)
    Object.data = mesh
    bpy.data.collections[collection].objects.link(Object)
    bpy.context.view_layer.objects.active = Object

#creates new object
def obj_new(Name, co, faces, collection):
    obj_mesh(co, faces, collection)
    bpy.data.objects["Obj"].name = Name 
    bpy.data.meshes[bpy.data.objects[Name].data.name].name = Name

#delete objects from list
def obj_del(List):
    bpy.ops.object.select_all(action='DESELECT')
    for o in List:
        obj[o].select_set(state=True)
        bpy.ops.object.delete()

#set active object and select objects from list
def active_ob(object, objects):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects[object].select_set(state=True)
    bpy.context.view_layer.objects.active = bpy.data.objects[object]
    if objects is not None:
        for o in objects:
            bpy.data.objects[o].select_set(state=True)

def add_parent(parent, children):
    active_ob(parent, children)
    bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)
    bpy.ops.object.select_all(action='DESELECT')

def adoption(parent, child, type, index):
    '''types: OBJECT, ARMATURE, LATTICE, VERTEX, VERTEX_3, BONE'''
    par = bpy.data.objects[parent]
    ch = bpy.data.objects[child]
    mw = ch.matrix_world
    ch.parent = par
    ch.matrix_world = mw #par.matrix_world @ par.matrix_world.inverted()
    ch.parent_type = type
    if type == 'VERTEX':
        ch.parent_vertices[index] = 1
    if type == 'BONE':
        ch.parent_bone = index

# ------------------------------------------------------------------------ 
#Create Target
def add_empty(Name, collection):
    obj_empty = bpy.data.objects.new(Name, None)
    bpy.data.collections[collection].objects.link(obj_empty)
    obj_empty.empty_display_size = 0.01

# ------------------------------------------------------------------------ 
#Create Capsule
def capsule_data(length, radius, cap_coord):
    sc = np.eye(3)
    sc[0][0] = radius/2
    sc[1][1] = radius/2
    sc[2][2] = length/4
    coord = np.array([[i[0], i[1], i[2]] for i in cap_coord])
    nc = coord @ sc
    return nc

def add_rd_capsule(Name, length, radius, collection):
    co = np.array([[0.38268, 0.0, 3.92388], [0.2706, 0.2706, 3.92388], [0.0, 0.38268, 3.92388], [-0.2706, 0.2706, 3.92388], [-0.38268, 0.0, 3.92388], [-0.2706, -0.2706, 3.92388], [-0.0, -0.38268, 3.92388], [0.2706, -0.2706, 3.92388], [0.70711, 0.0, 3.70711], [0.5, 0.5, 3.70711], [0.0, 0.70711, 3.70711], [-0.5, 0.5, 3.70711], [-0.70711, 0.0, 3.70711], [-0.5, -0.5, 3.70711], [-0.0, -0.70711, 3.70711], [0.5, -0.5, 3.70711], [0.92388, 0.0, 3.38268], [0.65328, 0.65328, 3.38268], [0.0, 0.92388, 3.38268], [-0.65328, 0.65328, 3.38268], [-0.92388, 0.0, 3.38268], [-0.65328, -0.65328, 3.38268], [-0.0, -0.92388, 3.38268], [0.65328, -0.65328, 3.38268], [1.0, 0.0, 3.0], [0.70711, 0.70711, 3.0], [0.0, 1.0, 3.0], [-0.70711, 0.70711, 3.0], [-1.0, 0.0, 3.0], [-0.70711, -0.70711, 3.0], [-0.0, -1.0, 3.0], [0.70711, -0.70711, 3.0], [0.0, 0.0, 4.0], [0.38268, 0.0, 0.07612], [0.2706, 0.2706, 0.07612], [0.0, 0.38268, 0.07612], [-0.2706, 0.2706, 0.07612], [-0.38268, 0.0, 0.07612], [-0.2706, -0.2706, 0.07612], [-0.0, -0.38268, 0.07612], [0.2706, -0.2706, 0.07612], [0.70711, 0.0, 0.29289], [0.5, 0.5, 0.29289], [0.0, 0.70711, 0.29289], [-0.5, 0.5, 0.29289], [-0.70711, 0.0, 0.29289], [-0.5, -0.5, 0.29289], [-0.0, -0.70711, 0.29289], [0.5, -0.5, 0.29289], [0.92388, 0.0, 0.61732], [0.65328, 0.65328, 0.61732], [0.0, 0.92388, 0.61732], [-0.65328, 0.65328, 0.61732], [-0.92388, 0.0, 0.61732], [-0.65328, -0.65328, 0.61732], [-0.0, -0.92388, 0.61732], [0.65328, -0.65328, 0.61732], [1.0, 0.0, 1.0], [0.70711, 0.70711, 1.0], [0.0, 1.0, 1.0], [-0.70711, 0.70711, 1.0], [-1.0, 0.0, 1.0], [-0.70711, -0.70711, 1.0], [-0.0, -1.0, 1.0], [0.70711, -0.70711, 1.0], [0.0, 0.0, 0.0]])
    faces = [(24, 25, 17, 16), (58, 57, 49, 50), (25, 26, 18, 17), (59, 58, 50, 51), (26, 27, 19, 18), (60, 59, 51, 52), (27, 28, 20, 19), (61, 60, 52, 53), (28, 29, 21, 20), (62, 61, 53, 54), (29, 30, 22, 21), (63, 62, 54, 55), (30, 31, 23, 22), (64, 63, 55, 56), (31, 24, 16, 23), (57, 64, 56, 49), (16, 17, 9, 8), (50, 49, 41, 42), (17, 18, 10, 9), (51, 50, 42, 43), (18, 19, 11, 10), (52, 51, 43, 44), (19, 20, 12, 11), (53, 52, 44, 45), (20, 21, 13, 12), (54, 53, 45, 46), (21, 22, 14, 13), (55, 54, 46, 47), (22, 23, 15, 14), (56, 55, 47, 48), (23, 16, 8, 15), (49, 56, 48, 41), (8, 9, 1, 0), (42, 41, 33, 34), (9, 10, 2, 1), (43, 42, 34, 35), (10, 11, 3, 2), (44, 43, 35, 36), (11, 12, 4, 3), (45, 44, 36, 37), (12, 13, 5, 4), (46, 45, 37, 38), (13, 14, 6, 5), (47, 46, 38, 39), (14, 15, 7, 6), (48, 47, 39, 40), (15, 8, 0, 7), (41, 48, 40, 33), (25, 24, 57, 58), (26, 25, 58, 59), (27, 26, 59, 60), (28, 27, 60, 61), (29, 28, 61, 62), (30, 29, 62, 63), (31, 30, 63, 64), (24, 31, 64, 57), (0, 1, 32), (1, 2, 32), (2, 3, 32), (3, 4, 32), (4, 5, 32), (5, 6, 32), (6, 7, 32), (7, 0, 32), (34, 33, 65), (35, 34, 65), (36, 35, 65), (37, 36, 65), (38, 37, 65), (39, 38, 65), (40, 39, 65), (33, 40, 65)]
    cor = capsule_data(length, radius, co)
    obj_new(Name, cor, faces, collection)
    try:
        bpy.ops.rigidbody.objects_add(type='ACTIVE')        
    except:
        pass

# ------------------------------------------------------------------------ 
#Constraints

def limit_bone_rotation(self, data):
    obj = bpy.data.objects
    pb = obj[bpy.context.object.name].pose.bone[self.name]
    bc = pb.constraints.new(type='LIMIT_ROTATION')
    bc.owner_space = 'LOCAL'
    bc.use_limit_x = True
    bc.use_limit_y = True
    bc.use_limit_z = True
    bc.min_x = radians(data[self.name][0])
    bc.max_x = radians(data[self.name][1])
    bc.min_y = radians(data[self.name][2])
    bc.max_y = radians(data[self.name][3])
    bc.min_z = radians(data[self.name][4])
    bc.max_z = radians(data[self.name][5])

#************************************************************ADD COPY TRANSFORMATION

def copy_empty_transform(pb, blist, infl):
    obj = bpy.data.objects
    for bo in pb:
        if bo.name in blist:
            joint = "RDjoint_{}".format(bo.name)
            bc = bo.constraints.new(type='COPY_TRANSFORMS')
            bc.target = obj[joint]
            bc.influence = infl
            
def copy_bone_transform(Dict):
    obj = bpy.data.objects
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

#************************************************************ADD INVERSE KINEMATICS

def add_IK(Dict, armature):
    for bo in bpy.context.object.pose.bones:
        if bo.name in Dict:
            nc = bo.constraints.new(type='IK')
            nc.target = bpy.data.objects[armature]
            nc.subtarget = Dict[bo.name]
            nc.influence = 0.5
            nc.use_rotation = True
            nc.use_location = True
            nc.chain_count = 1        

#************************************************************ADD CHILD OF CONSTRAINT

def add_child_of(object, Name, target, influence):
    cc = obj[object].constraints.new(type='CHILD_OF')
    cc.name = Name
    cc.target = obj[target]
    cc.influence = influence
    #bpy.ops.constraint.childof_set_inverse(constraint=Name, owner='OBJECT')

#************************************************************REMOVE CONSTRAINT

def remove_bone_constraints(self, constraint):
    obj = bpy.data.objects
    pb = obj[bpy.context.object.name].pose.bone[self.name]
    rbc = (r for r in pb.constraints if r.type == constraint)
    for r in rbc:
        pb.constraints.remove(r)

# ------------------------------------------------------------------------ 
#Modifiers

def add_modifier(Object, Name, Type):
    Object.modifiers.new(Name, type=Type)

def add_shrinkwrap(Object, Target):
    add_modifier(Object, 'shrink', 'SHRINKWRAP')
    oms = Object.modifiers['shrink']
    oms.target = Target

def apply_mod(Name):
    #credit to Jayanam
    active_obj = bpy.context.view_layer.objects.active
    for obj in bpy.context.view_layer.objects:
        for modifier in obj.modifiers:
            if Name in modifier.name:
                bpy.context.view_layer.objects.active = obj
                bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.context.view_layer.objects.active = active_obj

###############################################################################################################################
#RAGDOLL RIG

def new_ragdoll(armature, up_vec, extension, collection, container):
    '''Construct a ragdoll from existing skeleton'''
    sect = body_sections(container)
    obj = bpy.data.objects
    body = armature.children[0]
    pb = armature.pose.bones
    pbn = [i.name for i in pb]
    pbd = posebone_dict(armature)
    bd = bone_dict(bpy.data.armatures[0].bones)
    for i in container:
        if i in pbn:
            if i == container[0]: #head
                rad = pbd[i][4] * 3 #(bd[i][3]) * 3
            if i == container[1]: #neck
                rad = pbd[i][4] #(bd[i][3]) * 3
            if i == container[19]: #pelvis
                rad = pbd[i][4] #/ 2
            if i in sect[0]: #arms
                rad = (bd[i][3]) * .1
            if i in sect[1]: #legs
                rad = (bd[i][3]) * .1
            if i in sect[2]: #spine
                rad = (bd[i][3]) * 2
            else:
                rad = pbd[i][4] / 4
            RD = RagdollPart(i, np.array(pbd[i][1]), np.array(pbd[i][3]), up_vec, rad, extension, collection)
            loc = pbd[i][1]
            rot = pbd[i][8]
            RD.ragdollpart(loc, rot)
            part = extension + "_" + i
            joint = "RDjoint_" + i
            obj[part].matrix_world = bd[i][10] @ Matrix(rotation_matrix(radians(-90),0,0)).to_4x4()
            obj[joint].matrix_world = bd[i][10]
            RD.shrink_rd(body)

def add_rigid_body(collection): #weight, Dict
    obj = bpy.data.objects
    coll = bpy.data.collections
    joints = (i.name for i in coll[collection].objects if i.name.startswith("RDjoint_"))
    rd = (i.name for i in coll[collection].objects if not i.name.startswith("RDjoint_"))
    try:
        for r in rd:
            active_ob(r, None)
            bpy.ops.rigidbody.object_add()
            obj[r].rigid_body.mass = 0.4 #weight x part
            obj[r].rigid_body.collision_shape = 'MESH'
            obj[r].rigid_body.mesh_source = 'FINAL'
        for j in joints:
            active_ob(j, None)
            bpy.ops.rigidbody.constraint_add(type = 'GENERIC')
    except:
        pass

def add_rbc(part, Dict, extension):
    obj = bpy.data.objects
    coll = bpy.data.collections
    if not Dict[part][6]:
        pass
    else:
        #rg_list
        joint = "RDjoint_" + part
        p1 = extension + "_" + Dict[part][6]
        p2 = extension + "_" + part
        #rd_dict[part]
        xl = Dict[part][0]
        xu = Dict[part][1]
        yl = Dict[part][2]
        yu = Dict[part][3]
        zl = Dict[part][4]
        zu = Dict[part][5]
        rbc = obj[joint].rigid_body_constraint
        rbc.type = 'GENERIC'
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


def test_plane():
    bpy.ops.mesh.primitive_plane_add(size=2, enter_editmode=False, location=(0, 0, 0))
    bpy.ops.transform.resize(value=(3, 3, 3), orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    bpy.ops.transform.translate(value=(0, 0, -0.0526462), orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(False, False, True), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False)
    bpy.ops.rigidbody.object_add()
    bpy.context.object.rigid_body.type = 'PASSIVE'
    bpy.context.object.rigid_body.kinematic = True
    bpy.context.object.hide_render = True
    bpy.context.object.hide_viewport = True


###############################################################################################################################
#PHYSICS RIG

#Create Vertex Group
def add_vert_group(object, vgroup, index):
    nvg = bpy.data.objects[object].vertex_groups.new(name=vgroup)
    nvg.add(index, 1.0, "ADD") 

#Set Vertex Weight
def set_weight(object, index, weight):
    bpy.data.meshes[object].vertices[index].groups[0].weight = weight

#Create Rig
def rig_data(parts):
    bones = bpy.data.armatures[0].bones
    bd = bone_dict(bones)
    rd = [bd[p][2] for p in parts]
    last = parts[-1]
    rd += [ bd[last][6] ]
    return rd, [[i, i+1] for i in range(len(rd)-1)]

def add_rig(Name, parts):
    co, edges = rig_data(parts)
    obj_new(Name, co, edges, [], coll_rig)
    idx = [i for i in range(len(co))]
    add_vert_group(Name, Name, idx)


#Main Rig
def body_rig(parts):
    #Create edge mesh    
    for p in parts:
        add_rig(p, parts[p])
    bpy.ops.object.select_all(action='DESELECT')
    #add vertex weights
    for r in parts:
        vt = bpy.data.meshes[r].vertices
        idx = [i for i in range(len(parts[r]))]
        for w in idx:
            rp = parts[r][w]
            set_weight(r, w, ragdoll_dict[rp][8])

def hand_rig(parts):
    wts = [1, 0.25, 0.1, 0.1]
    for f in parts:
        add_rig(f, parts[f])
    bpy.ops.object.select_all(action='DESELECT')
    for r in parts:
        vt = bpy.data.meshes[r].vertices
        idx = [i for i in range(len(parts[r]))]
        for w in idx:
            set_weight(r, w, wts[w])

def chest_rig(parts):
    wts = [1, 0.25]
    for f in parts:
        add_rig(f, parts[f])
    bpy.ops.object.select_all(action='DESELECT')
    for r in parts:
        vt = bpy.data.meshes[r].vertices
        idx = [i for i in range(len(parts[r]))]
        for w in idx:
            set_weight(r, w, wts[w])


def rig_combine():
    rp = [r for r in rig_parts]
    cp = [c for c in chest_parts]
    lh = [l for l in left_hand_rig]
    rh = [r for r in right_hand_rig]
    child_list = rp[1:] + cp
    parent_ob = rp[0]
    l_parent = "left_arm"
    r_parent = "right_arm"
    for child in child_list:
        adoption(parent_ob, child, "OBJECT", '')
    for finger in lh:
        adoption(l_parent, finger, "OBJECT", '')
    for finger in rh:
        adoption(r_parent, finger, "OBJECT", '')

def rig_merge():
    rp = [r for r in rig_parts]
    cp = [c for c in chest_parts]
    lh = [l for l in left_hand_rig]
    rh = [r for r in right_hand_rig]
    child_list = rp[1:] + cp + lh + rh
    parent_ob = rp[0]
    bpy.ops.object.select_all(action='DESELECT')
    for child in child_list:
        obj[child].select_set(state=True)
    obj[parent_ob].select_set(state=True)
    bpy.context.view_layer.objects.active = obj[parent_ob]
    bpy.ops.object.join()
    bpy.ops.object.select_all(action='DESELECT')

def pino_kio(tr_to_list):
    for bo in bpy.context.object.pose.bones:
        if bo.name in tr_to_list:
            tt = pk_dict.get(bo.name)
            nc = bo.constraints.new(type='TRACK_TO')
            nc.target = bpy.data.objects[tt[3]]
            nc.influence = tt[0]
            nc.use_target_z = True
            nc.up_axis = tt[1]
            nc.track_axis = tt[2]

###############################################################################################################################
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

# ------------------------------------------------------------------------
#    Operators
# ------------------------------------------------------------------------

class OBJECT_OT_pino_kio_ragdoll(Operator):
    """Add a Ragdoll Rig to an Armature"""
    bl_idname = "object.pino_kio_ragdoll"
    bl_label = "Pino_Kio Ragdoll"
    bl_options = {'REGISTER', 'UNDO'}
        
    def execute(self, context):
        RD = Ragdoll(bpy.context.object, [0,0,1], "RD", "Ragdoll")
        RD.ragdoll(part_names)
        return {'FINISHED'}


class OBJECT_OT_test_plane(Operator):
    """Add a test invisible passive plane"""
    bl_idname = "object.test_plane"
    bl_label = "Test Plane"
    bl_options = {'REGISTER', 'UNDO'}
        
    def execute(self, context):
        test_plane()
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

        #layout.prop(rd_rig, "rd_influence")
        layout.operator("object.pino_kio_ragdoll")
        layout.operator("object.test_plane")
        layout.separator()



# ------------------------------------------------------------------------
#    Registration
# ------------------------------------------------------------------------

classes = (
    RR_Properties,
    OBJECT_OT_pino_kio_ragdoll,
    OBJECT_PT_pino_kio_ragdoll,
    OBJECT_OT_test_plane,
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