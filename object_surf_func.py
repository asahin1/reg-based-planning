import numpy as np
# from shapely.geometry import Polygon, LineString, MultiPoint
# from shapely.geometry import Point as sPoint
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint

def plot_3d_object(object_):
    r = Renderer()
    for surf in object_:
        r.add((object_[surf][0],'b',1))
        if len(object_[surf])>2:
            r.add((object_[surf][2],'r',1))

    r.show()

def get_neighbors(object_):
    neighbors = dict()
    for surf in object_:
        current_surface = object_[surf][0]
        current_normal = object_[surf][1]
        dummy_surfaces = object_.copy()
        dummy_surfaces.pop(surf)
        neighbors[surf] = dict()
        for n in dummy_surfaces:
            candidate_surface = dummy_surfaces[n][0]
            candidate_normal = dummy_surfaces[n][1]
            check = intersection(current_surface,candidate_surface)
            if check is not None:
                rotation_axis = candidate_normal.cross(current_normal)
                angle = candidate_normal.angle(current_normal)
                axis_position = gPoint((check[0].x+check[1].x)/2,(check[0].y+check[1].y)/2,(check[0].z+check[1].z)/2)
                neighbors[surf][n] = ((candidate_surface,rotation_axis,axis_position,angle))
    return neighbors

def unfold_surface(surface_dict,neighbors_dict,surf_idx,other,neighbor,show=False):
    unfolded_neighbors = list()
    p = Renderer()
    p.add((surface_dict[surf_idx][0],'r',1))
    current_normal = surface_dict[surf_idx][1]
    candidate_normal = other[1]
    angle = candidate_normal.angle(current_normal)
    A,B,C = neighbor[1]
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    D = np.array([[1,0,0,-neighbor[2][0]],[0,1,0,-neighbor[2][1]],[0,0,1,-neighbor[2][2]],[0,0,0,1]])
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(D)@np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x@D
    idx = other
    P_init = np.empty((4,0))
    for point in other[0].points:
        point_vec = np.array([point.x,point.y,point.z,1]).reshape(4,1)
        P_init = np.concatenate([P_init,point_vec],axis=1)
    normal_vec = np.array([other[0].points[-1].x+candidate_normal[0],other[0].points[-1].y+candidate_normal[1],other[0].points[-1].z+candidate_normal[2],1]).reshape(4,1)
    P_init = np.concatenate([P_init,normal_vec],axis=1)
    P_final = T@P_init
    new_points = list()
    for i in range(P_final.shape[1]-1):
        new_points.append(gPoint(np.round(P_final[:3,i],decimals=3)))
    
    nnormal = Vector(P_final[:3,-1]-P_final[:3,-2])
    nsurf = ConvexPolygon((new_points))
    if len(other)>2:
        G_init = np.empty((4,0))
        for point in other[2].points:
            point_vec = np.array([point.x,point.y,point.z,1]).reshape(4,1)
            G_init = np.concatenate([G_init,point_vec],axis=1)
        G_final = T@G_init
        new_goal = list()
        for i in range(G_final.shape[1]):
            new_goal.append(gPoint(np.round(G_final[:3,i],decimals=3)))
        ngoal = ConvexPolygon((new_goal))
        
        p.add((nsurf,'k',1))
        p.add((ngoal,'k',1))
        if show:
            p.add((other[0],'k',1))
            p.show()
        return (nsurf,nnormal,ngoal)

    else:
        p.add((nsurf,'k',1))
        if show:
            p.add((other[0],'k',1))
            p.show()
        return (nsurf,nnormal)

def unfold_object(current_surface,surface_dict,neighbors_dict):
    unfolded_surfaces = dict()
    surface_list = list(surface_dict.keys())
    unfolded_surfaces[current_surface] = surface_dict[current_surface]
    surface_list.remove(current_surface)
    open_list = [(neighbor,[current_surface]) for neighbor in neighbors_dict[current_surface]]
    while len(surface_list) > 0:
        item = open_list[0]
        parents = list()
        if item[0] in surface_list:
            folded_surf = surface_dict[item[0]]
            child = item[0]
            while len(item[1])>0:
                parent = item[1][-1]
                folded_surf = unfold_surface(surface_dict,neighbors_dict,item[1][-1],folded_surf,neighbors_dict[parent][child])
                child = item[1][-1]
                parents.append(item[1].pop())
            unfolded_surf = folded_surf
        else:
            open_list.pop(0)
            continue
        unfolded_surfaces[item[0]] = unfolded_surf
        
        surface_list.remove(item[0])
        parents.reverse()
        for neighbor in neighbors_dict[item[0]]:
            if neighbor in surface_list:
                open_list.append((neighbor,parents+[item[0]]))
            else:
                continue
        open_list.pop(0)
    return unfolded_surfaces