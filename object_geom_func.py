import numpy as np
from shapely.geometry import Polygon, LineString, MultiPoint
from shapely.geometry import Point as sPoint
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint
from finger_params import*

def generate_map(unfolded_surfaces,base,contact):
    goal_map = dict()
    surf_map = dict()
    main = unfolded_surfaces[base]
    translation = [-main[0].center_point.x,-main[0].center_point.y,-main[0].center_point.z]
    axis = Vector(0,0,1).cross(main[1])
    angle = Vector(0,0,1).angle(main[1])
    A,B,C = axis
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    if L == 0:
        R_y = np.eye(4)
    else:
        R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x
    for item in unfolded_surfaces:
        surf_init = unfolded_surfaces[item][0]
        P_init = np.empty((4,0))
        for point in surf_init.points:
            point_vec = np.array([point.x+translation[0],point.y+translation[1],point.z+translation[2],1]).reshape(4,1)
            P_init = np.concatenate([P_init,point_vec],axis=1)
        P_final = T@P_init
        new_points = list()
        for i in range(P_final.shape[1]):
            new_points.append(list(P_final[:2,i]))
        surfs = Polygon(new_points)
        surf_map[item] = surfs
        if len(unfolded_surfaces[item])>2:
            goal_init = unfolded_surfaces[item][2]
            P_init = np.empty((4,0))
            for point in goal_init.points:
                point_vec = np.array([point.x+translation[0],point.y+translation[1],point.z+translation[2],1]).reshape(4,1)
                P_init = np.concatenate([P_init,point_vec],axis=1)
            P_final = T@P_init
            new_points = list()
            for i in range(P_final.shape[1]):
                new_points.append(list(P_final[:2,i]))
            goal = Polygon(new_points)
            goal_map[item] = goal
    C_init = np.empty((4,0))
    for point in contact.points:
        point_vec = np.array([point.x+translation[0],point.y+translation[1],point.z+translation[2],1]).reshape(4,1)
        C_init = np.concatenate([C_init,point_vec],axis=1)
    C_final = T@C_init
    new_points = list()
    for i in range(C_final.shape[1]):
        new_points.append(list(C_final[:2,i]))
    contact_map = Polygon(new_points)
    
    return surf_map,goal_map,contact_map

def translate_point(point,vector):
    p = np.array([point.x,point.y,point.z])
    v = np.array([vector[0],vector[1],vector[2]])
    return gPoint(p+v)

def rotate_vector(vector,axis,angle):
    A,B,C = axis
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    if L == 0:
        R_y = np.eye(4)
    else:
        R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x
    C_init = np.array([vector[0],vector[1],vector[2],1]).reshape(4,1)
    C_final = T@C_init
    for i in range(C_final.shape[1]):
        new_points = np.round(C_final[:3,i],decimals=3)
    return Vector(new_points)

def place_finger(obj,d,z,normal,distal,hand_normal):
    for surf in obj:
        if obj[surf][1].angle(normal)<1e-3:
            surface = obj[surf][0]
            break
    for point in surface.points:
        for other_point in surface.points:
            if point==other_point:
                continue
            else:
                edge = Vector(point,other_point)
                angle = edge.angle(distal)
                if angle <1e-3:
                    obj_length = edge.length()
        
    finger_center = translate_point(surface.center_point,(hand_normal*z - distal*(d+obj_length/2-finger_w/2)))
    finger_p1 = translate_point(finger_center,(distal*(finger_w/2)+distal.cross(normal)*(finger_h/2)))
    finger_p2 = translate_point(finger_center,(-distal*(finger_w/2)+distal.cross(normal)*(finger_h/2)))
    finger_p3 = translate_point(finger_center,(-distal*(finger_w/2)-distal.cross(normal)*(finger_h/2)))
    finger_p4 = translate_point(finger_center,(distal*(finger_w/2)-distal.cross(normal)*(finger_h/2)))
    finger = ConvexPolygon((finger_p1,finger_p2,finger_p3,finger_p4))
    return finger

def pivot_finger(angle,poly,normal,center,distal):
    A,B,C = normal
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    D = np.array([[1,0,0,-center.x],[0,1,0,-center.y],[0,0,1,-center.z],[0,0,0,1]])
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    if L == 0:
        R_y = np.eye(4)
    else:
        R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(D)@np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x@D
    P_init = np.empty((4,0))
    for point in poly.points:
        point_vec = np.array([point.x,point.y,point.z,1]).reshape(4,1)
        P_init = np.concatenate([P_init,point_vec],axis=1)
    distal_vec = np.array([poly.points[-1].x+distal[0],poly.points[-1].y+distal[1],poly.points[-1].z+distal[2],1]).reshape(4,1)
    P_init = np.concatenate([P_init,distal_vec],axis=1)
    P_final = T@P_init
    new_points = list()
    for i in range(P_final.shape[1]-1):
        new_points.append(gPoint(np.round(P_final[:3,i],decimals=3)))
    
    ndistal = Vector(np.round(P_final[:3,-1]-P_final[:3,-2],decimals=3))
    nsurf = ConvexPolygon((new_points))
    return (nsurf,normal,ndistal)

def get_finger_param(finger_poly,obj):
    finger_center = finger_poly[0].center_point
    for surf in obj:
        if obj[surf][1].angle(finger_poly[1])<1e-3:
            surface = obj[surf][0]
            break
    for point in surface.points:
        for other_point in surface.points:
            if point==other_point:
                continue
            else:
                edge = Vector(point,other_point)
                angle = edge.angle(finger_poly[2])
                if angle <1e-3:
                    obj_length = edge.length()
    obj_center = surface.center_point
    close_edge = translate_point(obj_center,(-finger_poly[2]*(obj_length/2)))
    vec = Vector(finger_center,close_edge)
    if vec.length()==0:
        d = finger_w/2
        z = 0
    else:
        ang = vec.angle(finger_poly[2])
        if ang<np.pi/2:
            b = vec.length()*abs(np.cos(ang))
        else:
            b = -vec.length()*abs(np.cos(ang))
        d = np.round(b+finger_w/2,decimals=1)
        q = obj_center.distance(finger_center)**2-(b+obj_length/2)**2
        if q < 0 and abs(q) < 1e-3:
            q = 0
        if finger_poly[2].cross(finger_poly[1]).angle(vec)<np.pi/2:
            z = -np.round(np.sqrt(q),decimals=1)
        else:
            z = np.round(np.sqrt(q),decimals=1)
    if np.isnan(z):
        print("hey")
        print(obj_center.distance(finger_center))
        print((b+obj_length/2))
        print(obj_center)
        print(finger_center)
        print(b)
        print(vec)
        print(finger_poly[2])
        
        return None
    return d,z

def find_contact_center(finger_poly,normal,distal,surface,d,z,obj_l):
    if finger_w - d - obj_l > 0:
        v1 = distal*0
    else:
        v1 = (-(finger_w-d)/2 + (obj_l/2))*distal
    v_temp = Vector(surface.center_point,finger_poly.center_point)
    hand_normal = distal.cross(normal)
    if v_temp.length() == 0:
        b = 0
    else:
        ang = v_temp.angle(hand_normal)
        if ang<np.pi/2:
            b = v_temp.length()*abs(np.cos(ang))
        else:
            b = -v_temp.length()*abs(np.cos(ang))
    v2 = b*hand_normal
    center = translate_point(surface.center_point,v1+v2)
    return center