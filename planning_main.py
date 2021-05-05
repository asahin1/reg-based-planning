import numpy as np
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint
from shapely.geometry import Polygon, LineString, MultiPoint
from shapely.geometry import Point as sPoint
import scipy.optimize as opt
from math import*
import time
import os
import datetime
import queue
# from manip_objects import square_prism, rectangular_prism, rectangular_prism_tall_flanged, hexagonal_prism
# from hand_kinematics import solve_t1_slide_left, solve_t2_slide_right, calculate_th1, calculate_th2, action_left_equations, action_right_equations, theta_conversion
from object_definitions import get_OBJ
from object_geom_func import generate_map, translate_point, rotate_vector, place_finger, pivot_finger, get_finger_param, find_contact_center
from object_surf_func import plot_3d_object, get_neighbors, unfold_surface, unfold_object
from finger_params import*
from experiments import get_experimental_setup

def solve_t1_slide_left(variables, coords):
    solt2 = variables[0]
    eqn = coords[1]*cos(solt2) - coords[0]*sin(solt2) + FINGER_WIDTH
    return eqn

def calculate_th1(th2, d2):
    x_coord = PALM_WIDTH + (d2 + OBJECT_LENGTH / 2.0) * np.cos(th2) - (OBJECT_WIDTH/2.0 + FINGER_WIDTH) * np.sin(th2)
    y_coord = (d2 + OBJECT_LENGTH / 2.0) * np.sin(th2) + (OBJECT_WIDTH / 2.0 + FINGER_WIDTH) * np.cos(th2)
    n = len(X_vertices)
    R = np.array([[cos(-GAMMA + th2), -sin(-GAMMA + th2), x_coord],
                  [sin(-GAMMA + th2), cos(-GAMMA + th2), y_coord],
                  [0, 0, 1]])
    coords = np.dot(R, np.concatenate(([X_vertices], [Y_vertices], np.ones((1, n)))))

    beta = -9999
    contact_left = -1

    for i in range(n):
        initial_guess_t1 = np.pi / 2
        solution = opt.fsolve(solve_t1_slide_left, initial_guess_t1, args=coords[:, i], full_output=True)
        if (solution[2] == 1 and solution[0] > 0 and solution[0] < np.pi and solution[0] > beta):
            beta = solution[0]
            contact_left = i
    
    if beta == -9999:
        th1 = None
    else:
        th1 = beta[0]

    d1 = sqrt((coords[0, contact_left] - PALM_WIDTH) ** 2 + coords[1, contact_left] ** 2 - FINGER_WIDTH** 2)

    return th1

def solve_t2_slide_right(variables, coords):
    solt2 = variables[0]
    eqn = coords[1]*cos(solt2) - (coords[0] - PALM_WIDTH)*sin(solt2) - FINGER_WIDTH
    return eqn

def calculate_th2(th1, d1):
    x_coord = (d1 + OBJECT_LENGTH/2.0) * np.cos(th1) + (FINGER_WIDTH + OBJECT_WIDTH/2) * np.sin(th1)
    y_coord = (d1 + OBJECT_LENGTH/2.0) * np.sin(th1) - (FINGER_WIDTH + OBJECT_WIDTH/2) * np.cos(th1)
    n = len(X_vertices)
    R = np.array([[cos(-GAMMA + th1), -sin(-GAMMA + th1), x_coord], [sin(-GAMMA + th1), cos(-GAMMA + th1), y_coord], [0, 0, 1]])
    coords = np.dot(R, np.concatenate(([X_vertices], [Y_vertices], np.ones((1, n)))))

    beta = 99999
    contact_right = -1

    for i in range(n):
        initial_guess_t2 = np.pi / 2
        solution = opt.fsolve(solve_t2_slide_right, initial_guess_t2, args=coords[:, i], full_output=True)
        if (solution[2] == 1 and solution[0] > 0 and solution[0] < np.pi and solution[0] < beta):
            beta = solution[0]
            contact_right = i
    
    if beta == 99999:
        th2 = None
    else:
        th2 = beta[0]
    d2 = sqrt((coords[0, contact_right] - PALM_WIDTH) ** 2 + coords[1, contact_right] ** 2 - FINGER_WIDTH ** 2)

    return th2

def action_right_equations(variables) :
    (th1,th2) = variables
    eqn1 = FINGER_WIDTH*sin(th1)+FINGER_WIDTH*sin(th2)+left_position * cos(th1) + OBJECT_WIDTH * sin(th1) - PALM_WIDTH - right_position * cos(th2)
    eqn2 =-FINGER_WIDTH*cos(th1)-FINGER_WIDTH*cos(th2)+left_position * sin(th1) - OBJECT_WIDTH * cos(th1) - right_position * sin(th2)
    return [eqn1, eqn2]

def action_left_equations(variables) :
    (th1, th2) = variables
    eqn1 = FINGER_WIDTH * sin(th1) + FINGER_WIDTH * sin(th2) + left_position * cos(th1) + OBJECT_WIDTH * sin(th2) - PALM_WIDTH - right_position * cos(th2)
    eqn2 = -FINGER_WIDTH * cos(th1) - FINGER_WIDTH * cos(th2) + left_position * sin(th1) - OBJECT_WIDTH * cos(th2) - right_position * sin(th2)
    return [eqn1, eqn2]

def theta_conversion(left, right, action_name):
    global left_position
    global right_position

    left_position =left
    right_position=right
    if (action_name == "Slide right up" or action_name == "Slide right down"):
        for i in range(31):
            initial_guess=(i/10.0,i/10.0)
            solution = opt.fsolve(action_right_equations, initial_guess, full_output=True)
            if solution[2]==1 and solution[0][0]>0 and solution[0][0]<np.pi and solution[0][1]<np.pi and solution[0][1]>0:
                return solution[0]

        return (None,None)
    
    elif (action_name == "Slide left up" or action_name == "Slide left down"):
        for i in range(31):
            initial_guess=(i/10.0,i/10.0)
            solution = opt.fsolve(action_left_equations, initial_guess, full_output=True)
            if solution[2] == 1 and solution[0][0] > 0 and solution[0][0] < np.pi and solution[0][1] < np.pi and solution[0][1] > 0:
                return solution[0]
            
        return (None,None)
    
    # Object size is used
    elif (action_name=="Rotate ccw"):
        solution= np.pi - np.arccos((((right_position)**2 + OBJECT_WIDTH**2 - PALM_WIDTH**2 - 
                                      (left_position)**2)/(2*PALM_WIDTH*(left_position))))
        return (solution)

    elif (action_name=="Rotate cw"):
        solution= np.arccos(((left_position)**2 + OBJECT_WIDTH**2 - 
                             (right_position)**2 - PALM_WIDTH**2)/(2*PALM_WIDTH*(right_position)))
        return (solution)

class Node:
    def __init__(self,finger_l,finger_r,parent=None,resolution=0.5,action=None):
        self.parent = parent
        self.action = action
        self.finger_l = finger_l
        self.finger_r = finger_r
        self.resolution = resolution
        self.hand_normal = finger_l[3].cross(finger_l[2])
        self.finger_l_poly = place_finger(OBJ,finger_l[0],finger_l[1],finger_l[2],finger_l[3],self.hand_normal)
        self.finger_r_poly = place_finger(OBJ,finger_r[0],finger_r[1],finger_r[2],finger_r[3],self.hand_normal)
        for surf in OBJ:
            check = intersection(self.finger_l_poly,OBJ[surf][0])
#             if check is not None and OBJ[surf][1].angle(finger_l[2])<1e-3:
            if OBJ[surf][1].angle(finger_l[2])<1e-3:
                self.surf_l = surf
                self.c_l = check
        try:
#             print(self.surf_l)
            pass
        except:
            self.plot_state()
            for surf in OBJ:
                print("Finger:",self.finger_l_poly)
                print("Surf:",OBJ[surf][0])
                check = intersection(self.finger_l_poly,OBJ[surf][0])
                print(check)
                
        for surf in OBJ:
            check = intersection(self.finger_r_poly,OBJ[surf][0])
#             if check is not None and OBJ[surf][1].angle(finger_r[2])<1e-3:
            if OBJ[surf][1].angle(finger_r[2])<1e-3:
                self.surf_r = surf
                self.c_r = check
        self.pivot_side = OBJ[self.surf_l][0]
        for a in self.pivot_side.segments():
            for b in self.pivot_side.segments():
                if a == b:
                    continue
                else:
                    if a.intersection(b) is not None:
                        self.pivot_angle = Vector(a[0],a[1]).angle(Vector(b[0],b[1]))
                        break
        a,b,c = self.hand_normal
        axis = self.hand_normal.cross(Vector(0,0,1))
        angle = self.hand_normal.angle(Vector(0,0,1))
        points = []
        for surf in OBJ:
            for point in OBJ[surf][0].points:
                t = (-a*point.x - b*point.y - c*point.z)/(a**2 + b**2 + c**2)
                xn = point.x + a*t
                yn = point.y + b*t
                zn = point.z + c*t
                vec = rotate_vector(Vector(xn,yn,zn),axis,angle)
                x,y,z = vec
                points.append(sPoint(x,y))
        mpoints = MultiPoint(points)
        global X_vertices
        global Y_vertices
        self.x_vertices,self.y_vertices = mpoints.convex_hull.exterior.xy
#         self.cross_section = list(zip(x[:-1],y[:-1]))
        X_vertices = self.x_vertices[:-1]
        Y_vertices = self.y_vertices[:-1]
        self.angle_cw_l = None
        
        for neighbor in OBJ_neighbors[self.surf_l]:
            if OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal).length()<1e-3:
                try:
                    q = self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)
                except:
                    ac = self.finger_l[2].cross(OBJ[neighbor][1])*self.hand_normal/(self.finger_l[2].cross(OBJ[neighbor][1]).length()*self.hand_normal.length())
                    if ac < -1:
                        q = np.pi
                    if ac > 1:
                        q = 0
                    
                if q<1e-3:
                    self.angle_cw_l = self.finger_l[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("cw: ", self.angle_cw_l)
                if abs(q-np.pi)<1e-3:
                    self.angle_ccw_l = self.finger_l[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("ccw: ", self.angle_ccw_l)
        
        for neighbor in OBJ_neighbors[self.surf_r]:
            if OBJ_neighbors[self.surf_r][neighbor][1].cross(self.hand_normal).length()<1e-3:
                try:
                    q = self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)
                except:
                    ac = self.finger_r[2].cross(OBJ[neighbor][1])*self.hand_normal/(self.finger_r[2].cross(OBJ[neighbor][1]).length()*self.hand_normal.length())
                    if ac < -1:
                        q = np.pi
                    if ac > 1:
                        q = 0
                if q<1e-3:
                    self.angle_cw_r = self.finger_r[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("cw: ", self.angle_cw_r)
                if abs(q-np.pi)<1e-3:
                    self.angle_ccw_r = self.finger_r[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("ccw: ", self.angle_ccw_r)
        
        
#         for neighbor in OBJ_neighbors[self.surf_l]:
# #             print(self.action)
# #             print(self.finger_l[2],square[neighbor][1],self.hand_normal)
# #             print(self.finger_l[2].cross(square[neighbor][1]).angle(self.hand_normal))
#             if OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal).length()<1e-3:
#                 try:
#                     q = self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)
#                 except:
#                     q = 0
                    
                    
#                 if self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)<1e-3:
#                     self.angle_cw_l = self.finger_l[2].angle(OBJ[neighbor][1])
# #                     print("Neighbor: ", neighbor)
# #                     print("cw: ", self.angle_cw_l)
#                 if abs(self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)-np.pi)<1e-3:
#                     self.angle_ccw_l = self.finger_l[2].angle(OBJ[neighbor][1])
# #                     print("Neighbor: ", neighbor)
# #                     print("ccw: ", self.angle_ccw_l)
        
#         for neighbor in OBJ_neighbors[self.surf_r]:
#             if OBJ_neighbors[self.surf_r][neighbor][1].cross(self.hand_normal).length()<1e-3:
#                 if self.finger_r[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)<1e-3:
#                     self.angle_cw_r = self.finger_r[2].angle(OBJ[neighbor][1])
# #                     print("Neighbor: ", neighbor)
# #                     print("cw: ", self.angle_cw_r)
#                 if abs(self.finger_r[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)-np.pi)<1e-3:
#                     self.angle_ccw_r = self.finger_r[2].angle(OBJ[neighbor][1])
# #                     print("Neighbor: ", neighbor)
# #                     print("ccw: ", self.angle_ccw_r)

        global GAMMA
        if self.angle_cw_l == None:
            print(self.surf_l)
            for neighbor in OBJ_neighbors[self.surf_l]:
                print("edge vector: ",OBJ_neighbors[self.surf_l][neighbor][1])
                print("hand normal: ",self.hand_normal)
                print("cross: ",OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal))
                if OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal).length()<1e-3:
                    print(self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal))
                    if self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)<1e-3:
                        self.angle_cw_l = self.finger_l[2].angle(OBJ[neighbor][1])
                    if abs(self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)-np.pi)<1e-3:
                        self.angle_ccw_l = self.finger_l[2].angle(OBJ[neighbor][1])
            current = self
            while current is not None:
                print(current.action)
                current = current.parent
            
        
        if self.surf_l==self.surf_r:
            print("Error")
            current = self
            while current is not None:
                print(current.action)
                print(current.finger_l[2],current.finger_l[3])
                print(current.finger_r[2],current.finger_r[3])
                current = current.parent
            
        
        for point in OBJ[self.surf_l][0].points:
            for other_point in OBJ[self.surf_l][0].points:
                if point==other_point:
                    continue
                else:
                    edge = Vector(point,other_point)
                    angle = edge.angle(finger_l[3])
                    if angle<1e-3:
                        self.objleft_l = edge.length()
                    if abs(angle- np.pi/2)<1e-3:
                        self.objleft_h = edge.length()
        
#         print("hey")
        for point in OBJ[self.surf_r][0].points:
            for other_point in OBJ[self.surf_r][0].points:
                if point==other_point:
                    continue
                else:
                    edge = Vector(point,other_point)
                    angle = edge.angle(finger_r[3])
#                     print(angle)
                    if angle<1e-3:
                        self.objright_l = edge.length()
                    if abs(angle - np.pi/2)<1e-3:
                        self.objright_h = edge.length()
#         print("oy")
        
        for point in OBJ[self.surf_l][0].points:
            for other_point in OBJ[self.surf_r][0].points:
                if point==other_point:
                    continue
                else:
                    edge = Vector(point,other_point)
                    angle = edge.angle(finger_r[2])
                    if angle <1e-3:
                        self.obj_w = edge.length()
                        break
                        
        self.map_l = unfold_object(self.surf_l,OBJ,OBJ_neighbors)
        self.map_r = unfold_object(self.surf_r,OBJ,OBJ_neighbors)
#         self.goal_list_l,self.contact_l = generate_map(self.map_l,self.surf_l,self.c_l)
#         self.goal_list_r,self.contact_r = generate_map(self.map_r,self.surf_r,self.c_r)
        self.surf_list_l,self.goal_list_l,self.sfing_l = generate_map(self.map_l,self.surf_l,self.finger_l_poly)
        self.surf_list_r,self.goal_list_r,self.sfing_r = generate_map(self.map_r,self.surf_r,self.finger_r_poly)
#         print(self.surf_list_l)
#         print(self.goal_list_l)
#         for surf in self.surf_list_l:
#             if surf.intersection(self.sfing_l).area>0:
#                 self.contact_l = surf.intersection(self.sfing_l)
#         for surf in self.surf_list_r:
#             if surf.intersection(self.sfing_r).area>0:
#                 self.contact_r = surf.intersection(self.sfing_r)
                
        
            
        self.contact_l = self.surf_list_l[self.surf_l].intersection(self.sfing_l)
        self.contact_r = self.surf_list_r[self.surf_r].intersection(self.sfing_r)
        global OBJECT_LENGTH
        global OBJECT_WIDTH
        # OBJECT_LENGTH = self.objlef
        OBJECT_WIDTH = self.obj_w
        self.obj_h = self.objright_h
        self.g = 0
        self.h = 0
        self.f = 0
        self.theta = [np.pi/2,np.pi/2]
        self.command = []
        action_list = ["Slide left down","Slide left up","Slide right down","Slide right up",
                       "Rotate ccw","Rotate cw","Move down","Move up","Pivot"]
        
        self.available_actions = list()
        

        if self.action is not None and self.action == "Pivot":
            pass
        else:
            for action in action_list:

                if action == "Slide left down":
                    d_l = self.finger_l[0] - resolution
                    d_r = self.finger_r[0]
                    if d_l > FINGER_END or d_l < FINGER_START:
                        continue
                    else:
                        sol=theta_conversion(d_l, d_r, action)
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        TH2_MAX = calculate_th2(TH1_MAX, d_l)
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        TH1_MIN = calculate_th1(TH2_MIN, d_r)
                        th1=sol[0]
                        th2 = sol[1]
                        if th1==None or th2==None:
                            continue
                        else:
                            if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                                pass
                            else:
                                continue
                        self.available_actions.append(action)
                elif action == "Slide left up":
                    d_l = self.finger_l[0] + resolution
                    d_r = self.finger_r[0]
                    if d_l > FINGER_END or d_l < FINGER_START:
                        continue
                    else:
                        sol=theta_conversion(d_l, d_r, action)
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        TH2_MAX = calculate_th2(TH1_MAX, d_l)
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        TH1_MIN = calculate_th1(TH2_MIN, d_r)
                        th1=sol[0]
                        th2 = sol[1]
                        if th1==None or th2==None:
                            continue
                        else:
                            if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                                pass
                            else:
                                continue
                        self.available_actions.append(action)
                elif action == "Slide right down":
                    d_r = self.finger_r[0] - resolution
                    d_l = self.finger_l[0]
                    if d_r > FINGER_END or d_r < FINGER_START:
                        continue
                    else:
                        sol=theta_conversion(d_l, d_r, action)
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        TH2_MAX = calculate_th2(TH1_MAX, d_l)
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        TH1_MIN = calculate_th1(TH2_MIN, d_r)
                        th1=sol[0]
                        th2 = sol[1]
                        if th1==None or th2==None:
                            continue
                        else:
                            if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                                pass
                            else:
                                continue
                        self.available_actions.append(action)
                elif action == "Slide right up":
                    d_r = self.finger_r[0] + resolution
                    d_l = self.finger_l[0]
                    if d_r > FINGER_END or d_r < FINGER_START:
                        continue
                    else:
                        sol=theta_conversion(d_l, d_r, action)
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        TH2_MAX = calculate_th2(TH1_MAX, d_l)
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        TH1_MIN = calculate_th1(TH2_MIN, d_r)
                        th1=sol[0]
                        th2 = sol[1]
                        if th1==None or th2==None:
                            continue
                        else:
                            if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                                pass
                            else:
                                continue
                        self.available_actions.append(action)
                elif action == "Rotate ccw":
    #                 if self.angle_ccw_l!=self.angle_ccw_r:
    #                     print("Different angles")
    #                     print(self.angle_ccw_l,self.angle_ccw_r)

                    rot_angle = self.angle_ccw_l
    #                 print(rot_angle)
                    axis_l = self.finger_l[3].cross(self.finger_l[2])
                    normal_l = rotate_vector(self.finger_l[2],axis_l,-rot_angle,)
                    distal_l = rotate_vector(self.finger_l[3],axis_l,-rot_angle)
                    axis_r = self.finger_r[3].cross(self.finger_r[2])
                    normal_r = rotate_vector(self.finger_r[2],axis_r,rot_angle)
                    distal_r = rotate_vector(self.finger_r[3],axis_r,rot_angle)
                    for surf in OBJ:
    #                     print(square[surf][1].angle(normal_l))
                        if OBJ[surf][1].angle(normal_l)<1e-3:
                            surf_l = surf
                            break
                    for surf in OBJ:
                        if OBJ[surf][1].angle(normal_r)<1e-3:
                            surf_r = surf
                            break
                    for point in OBJ[surf_l][0].points:
                        for other_point in OBJ[surf_l][0].points:
                            if point==other_point:
                                continue
                            else:
                                edge = Vector(point,other_point)
                                angle = edge.angle(distal_l)
                                if angle<1e-3:
                                    length_l = edge.length()
                    for point in OBJ[surf_r][0].points:
                        for other_point in OBJ[surf_r][0].points:
                            if point==other_point:
                                continue
                            else:
                                edge = Vector(point,other_point)
                                angle = edge.angle(distal_r)
                                if angle<1e-3:
                                    length_r = edge.length()
                    d_l = self.finger_l[0] + length_l
                    d_r = self.finger_r[0] - length_r
                    if d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START:
                        continue
                    else:
    #                     th1=theta_conversion(self.finger_l[0], self.finger_r[0], action)
                        th1=theta_conversion(d_l, d_r, action)
    #                     th2=calculate_th2(th1,self.finger_l[0])
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        th2=calculate_th2(th1,d_l)
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        TH2_MAX=calculate_th2(TH1_MAX,self.finger_l[0])
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        TH1_MIN=calculate_th1(TH2_MIN,self.finger_r[0])
                        if th1==None or th2==None:
                            continue
                        else:
                            if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                                pass
                            else:
                                continue
                        self.available_actions.append(action)
                elif action == "Rotate cw":
    #                 if self.angle_cw_l!=self.angle_cw_r:
    #                     print("Different angles")
    #                     print(self.angle_cw_l,self.angle_cw_r)
                    rot_angle = self.angle_cw_l
                    axis_l = self.finger_l[3].cross(self.finger_l[2])
                    normal_l = rotate_vector(self.finger_l[2],axis_l,rot_angle)
                    distal_l = rotate_vector(self.finger_l[3],axis_l,rot_angle)
                    axis_r = self.finger_r[3].cross(self.finger_r[2])
                    normal_r = rotate_vector(self.finger_r[2],axis_r,-rot_angle)
                    distal_r = rotate_vector(self.finger_r[3],axis_r,-rot_angle)
                    for surf in OBJ:
                        if OBJ[surf][1].angle(normal_l)<1e-3:
                            surf_l = surf
                            break
                    for surf in OBJ:
                        if OBJ[surf][1].angle(normal_r)<1e-3:
                            surf_r = surf
                            break
                    for point in OBJ[surf_l][0].points:
                        for other_point in OBJ[surf_l][0].points:
                            if point==other_point:
                                continue
                            else:
                                edge = Vector(point,other_point)
                                angle = edge.angle(distal_l)
                                if angle <1e-3:
                                    length_l = edge.length()
                    for point in OBJ[surf_r][0].points:
                        for other_point in OBJ[surf_r][0].points:
                            if point==other_point:
                                continue
                            else:
                                edge = Vector(point,other_point)
                                angle = edge.angle(distal_r)
                                if angle <1e-3:
                                    length_r = edge.length()
                    d_l = self.finger_l[0] - length_l
                    d_r = self.finger_r[0] + length_r
                    if d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START:
                        continue
                    else:
    #                     th2=theta_conversion(self.finger_l[0], self.finger_r[0], action)
                        th2=theta_conversion(d_l, d_r, action)
    #                     th1 = calculate_th1(th2, self.finger_r[0])
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        th1 = calculate_th1(th2, d_r)
                        OBJECT_LENGTH = self.objleft_l
                        GAMMA = self.angle_ccw_l
                        TH2_MAX = calculate_th2( TH1_MAX,self.finger_l[0])
                        OBJECT_LENGTH = self.objright_l
                        GAMMA = self.angle_cw_l
                        TH1_MIN = calculate_th1(TH2_MIN,self.finger_r[0])
                        if th1==None or th2==None:
                            continue
                        else:
                            if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                                pass
                            else:
                                continue
                        self.available_actions.append(action)
                elif action == "Move down":
                    z = self.finger_l[1] - resolution
                    # if z > self.obj_h/2 - finger_h/2 or z < -self.obj_h/2 + finger_h/2:
                    # if z > self.obj_h/2 or z < 4-self.obj_h/2:
                    if z > self.obj_h/2 - finger_h/2 or z < 4-self.obj_h/2:
                        continue
                    else:
                        self.available_actions.append(action)
                elif action == "Move up":
                    z = self.finger_l[1] + resolution
                    # if z > self.obj_h/2 - finger_h/2 or z < -self.obj_h/2 + finger_h/2:
                    # if z > self.obj_h/2 or z < -self.obj_h/2:
                    if z > self.obj_h/2 - finger_h/2 or z < 4-self.obj_h/2:
                        continue
                    else:
                        self.available_actions.append(action)
                elif action == "Pivot":
                    if z < 6-self.obj_h/2:
                        continue
                    pivot_angle = self.pivot_angle
                    left_center = find_contact_center(self.finger_l_poly,self.finger_l[2],self.finger_l[3],OBJ[self.surf_l][0],self.finger_l[0],self.finger_l[1],self.objleft_l)
                    right_center = find_contact_center(self.finger_r_poly,self.finger_r[2],self.finger_r[3],OBJ[self.surf_r][0],self.finger_r[0],self.finger_r[1],self.objright_l)
    #                 print("real, calc left:",(self.c_l.center_point,left_center))
    #                 print("real, calc right:",(self.c_r.center_point,right_center))
                    finger_poly_left = pivot_finger(pivot_angle,self.finger_l_poly,self.finger_l[2],left_center,self.finger_l[3])
                    finger_poly_right = pivot_finger(-pivot_angle,self.finger_r_poly,self.finger_r[2],right_center,self.finger_r[3])
                    d_l,z = get_finger_param(finger_poly_left,OBJ)
                    d_r,z = get_finger_param(finger_poly_right,OBJ)
                    # if z > self.obj_h/2 - finger_h/2 or z < -self.obj_h/2 + finger_h/2 or d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START:
                    # if z > self.obj_h/2 or z < -self.obj_h/2 or d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START:
                    new_obj_h = self.objleft_l
                    new_obj_l = self.objleft_h
                    if z > new_obj_h/2 or z < -new_obj_h/2 or d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START or d_l+new_obj_l<finger_w or d_r+new_obj_l<finger_w:
                        continue
                    else:
                        self.available_actions.append(action)
                
        if(self.action=="Slide left down"):
            (self.theta)= theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Slide left up"):
            (self.theta) = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Slide right down"):
            (self.theta) = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Slide right up"):
            (self.theta) = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        # Reverting back to previous d
        elif(self.action=="Rotate ccw"):
#             self.theta[0] = theta_conversion(self.parent.finger_l[0], self.parent.finger_r[0], self.action)
            self.theta[0] = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            OBJECT_LENGTH = self.objleft_l
            GAMMA = self.angle_ccw_l
            self.theta[1] = calculate_th2(self.theta[0], self.finger_l[0])
#             self.theta[0] = theta_conversion(self.finger_l[0]-OBJECT_SIZE, self.finger_r[0]+OBJECT_SIZE, self.action)
#             self.theta[1] = calculate_th2(self.theta[0], self.finger_l[0]-OBJECT_SIZE)
            self.command = self.theta

        elif(self.action=="Rotate cw"):
#             self.theta[1] = theta_conversion(self.parent.finger_l[0], self.parent.finger_r[0], self.action)
            self.theta[1] = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            OBJECT_LENGTH = self.objright_l
            GAMMA = self.angle_cw_l
            self.theta[0] = calculate_th1(self.theta[1], self.finger_r[0])
#             self.theta[1] = theta_conversion(self.finger_l[0]+OBJECT_SIZE, self.finger_r[0]-OBJECT_SIZE, self.action)
#             self.theta[0] = calculate_th1(self.theta[1], self.finger_r[0]+OBJECT_SIZE)
            self.command = self.theta

        elif (self.action=="Move down"):
            self.theta = self.parent.theta
            # self.command = self.finger_l[1]
            self.command = [self.parent.theta[0],self.parent.theta[1],resolution]
            
        elif (self.action=="Move up"):
            self.theta = self.parent.theta
            # self.command = self.finger_l[1]
            self.command = [self.parent.theta[0],self.parent.theta[1],resolution]

        elif (self.action=="Pivot"):
#             start_p = translate_point(OBJ[self.parent.surf_l][0].center_point,-self.parent.finger_l[3]*(self.parent.finger_l[0]+self.parent.objleft_l/2))
#             end_p = translate_point(OBJ[self.surf_l][0].center_point,-self.finger_l[3]*(self.finger_l[0]+self.objleft_l/2))
#             vec = Vector(start_p,end_p)
            
            self.theta=self.parent.theta
            
            if finger_w - self.parent.finger_l[0] - self.parent.objleft_l > 0:
                xl = self.parent.objleft_l/2
            else:
                xl = (finger_w - self.parent.finger_l[0])/2
            
#             print("xl:",xl)
#             print("o_l:",self.parent.objleft_l)
#             print("f_l:",finger_w)
#             print("d_l:",self.parent.finger_l[0])
#             print("z_l:",self.parent.finger_l[1])
            
            # Command: theta_l, d_l+xl, theta_pivot, z
            self.command = [self.parent.theta[0],self.parent.finger_l[0]+xl,self.parent.pivot_angle,self.parent.finger_l[1]]

    def __eq__(self,other):
        return (self.finger_l[0]==other.finger_l[0] and self.finger_r[0]==other.finger_r[0] and self.finger_l[1]==other.finger_l[1] and self.finger_r[1]==other.finger_r[1]
                    and self.surf_l==other.surf_l)

    def __lt__(self,other):
        return (self.f < other.f)


    def at_goal(self):
        flag_l = False
        try:
            if abs(self.goal_list_l[self.surf_l].intersection(self.contact_l).area-self.contact_l.area)<1e-3:
                    flag_l = True
        except:
            pass
        flag_r = False
        try:
            if abs(self.goal_list_r[self.surf_r].intersection(self.contact_r).area-self.contact_r.area)<1e-3:
                    flag_r = True
        except:
            pass
#         flag_l = False
#         for goal in self.goal_list_l:
#             if abs(goal.intersection(self.contact_l).area-self.contact_l.area)<1e-3:
#                 flag_l = True
#                 break
#         flag_r = False
#         for goal in self.goal_list_r:
#             if abs(goal.intersection(self.contact_r).area-self.contact_r.area)<1e-3:
#                 flag_r = True
#                 break
#         flag_l = False
#         for goal in self.goal_list_l:
#             if goal.contains(self.contact_l):
#                 flag_l = True
#                 break
#         flag_r = False
#         for goal in self.goal_list_r:
#             if goal.contains(self.contact_r):
#                 flag_r = True
#                 break
#         flag_l = False
#         for goal in self.goal_list_l:
#             if not goal.disjoint(self.contact_l):
#                 flag_l = True
#                 break
#         flag_r = False
#         for goal in self.goal_list_r:
#             if not goal.disjoint(self.contact_r):
#                 flag_r = True
#                 break
        return flag_l and flag_r
    
    def plot_state(self):
        
        r = Renderer()
        for surf in OBJ:
            r.add((OBJ[surf][0],'b',1))
            if len(OBJ[surf])>2:
                r.add((OBJ[surf][2],'r',1))

            if len(OBJ[surf])>3:
                r.add((OBJ[surf][3],'g',1))
        r.add((self.finger_l_poly,'k',1))
        r.add((self.finger_r_poly,'k',1))
        r.add((gPoint(-15,-15,-15),'k',1))
#         r.add((gPoint(15,15,15),'k',1))
        r.show()

def search(start_node,cost_list):
    start_time = time.time()
#     yet_to_visit_list = []
    yet_to_visit_list = queue.PriorityQueue()
    visited_list = []
#     yet_to_visit_list.append(start_node)
    yet_to_visit_list.put((start_node.f,start_node))
    outer_iterations = 0
    max_iterations = 100000

    while not yet_to_visit_list.empty():
        outer_iterations += 1
        if outer_iterations%1000 == 0:
            print("{}/{}".format(outer_iterations,max_iterations))

#         current_node = yet_to_visit_list[0]
#         current_index = 0
        current_node = yet_to_visit_list.get()[1]
        # print("Iteration: ",outer_iterations)
        # print("Prev action: ", current_node.action)
        # print("Available actions: ",current_node.available_actions)
        # if current_node.action == "Move down" or current_node.action == "Move up":
        #     print(current_node.action)
        # print("Action: {}, g={}, h={}, f={}".format(current_node.action,current_node.g,current_node.h,current_node.f))
#         time.sleep(3)

#         for index,item in enumerate(yet_to_visit_list):
#             if item.f < current_node.f:
#                 current_node = item
#                 current_index = index

        if outer_iterations > max_iterations:
            end_time = time.time()
            dt = end_time - start_time
            print("Time taken: {} s".format(dt))
            print("giving up on pathfinding too many iterations")
            path, actions = return_path(current_node)
            return path, actions, dt

#         yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)
        if current_node.at_goal():
            end_time = time.time()
            dt = end_time - start_time
            print("Time taken: {} s".format(dt))
            path, actions = return_path(current_node)
            return path, actions, dt

        children = []
        for action in current_node.available_actions:
            if action == "Slide left down":
                d_l = current_node.finger_l[0] - current_node.resolution
                d_r = current_node.finger_r[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Slide left up":
                d_l = current_node.finger_l[0] + current_node.resolution
                d_r = current_node.finger_r[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Slide right down":
                d_r = current_node.finger_r[0] - current_node.resolution
                d_l = current_node.finger_l[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Slide right up":
                d_r = current_node.finger_r[0] + current_node.resolution
                d_l = current_node.finger_l[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Rotate ccw":
                rot_angle = current_node.angle_ccw_l
                axis_l = current_node.finger_l[3].cross(current_node.finger_l[2])
                normal_l = rotate_vector(current_node.finger_l[2],axis_l,-rot_angle)
                distal_l = rotate_vector(current_node.finger_l[3],axis_l,-rot_angle)
                axis_r = current_node.finger_r[3].cross(current_node.finger_r[2])
                normal_r = rotate_vector(current_node.finger_r[2],axis_r,rot_angle)
                distal_r = rotate_vector(current_node.finger_r[3],axis_r,rot_angle)
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_l)<1e-3:
                        surf_l = surf
                        break
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_r)<1e-3:
                        surf_r = surf
                        break
                for point in OBJ[surf_l][0].points:
                    for other_point in OBJ[surf_l][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_l)
                            if angle<1e-3:
                                length_l = edge.length()
                for point in OBJ[surf_r][0].points:
                    for other_point in OBJ[surf_r][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_r)
                            if angle<1e-3:
                                length_r = edge.length()
                d_l = current_node.finger_l[0] + length_l
                d_r = current_node.finger_r[0] - length_r
                finger_l = (d_l,current_node.finger_l[1],normal_l,distal_l)
                finger_r = (d_r,current_node.finger_r[1],normal_r,distal_r)
            elif action == "Rotate cw":
                rot_angle = current_node.angle_cw_l
                axis_l = current_node.finger_l[3].cross(current_node.finger_l[2])
                normal_l = rotate_vector(current_node.finger_l[2],axis_l,rot_angle)
                distal_l = rotate_vector(current_node.finger_l[3],axis_l,rot_angle)
                axis_r = current_node.finger_r[3].cross(current_node.finger_r[2])
                normal_r = rotate_vector(current_node.finger_r[2],axis_r,-rot_angle)
                distal_r = rotate_vector(current_node.finger_r[3],axis_r,-rot_angle)
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_l)<1e-3:
                        surf_l = surf
                        break
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_r)<1e-3:
                        surf_r = surf
                        break
                for point in OBJ[surf_l][0].points:
                    for other_point in OBJ[surf_l][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_l)
                            if angle<1e-3:
                                length_l = edge.length()
                for point in OBJ[surf_r][0].points:
                    for other_point in OBJ[surf_r][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_r)
                            if angle<1e-3:
                                length_r = edge.length()
                d_l = current_node.finger_l[0] - length_l
                d_r = current_node.finger_r[0] + length_r
                finger_l = (d_l,current_node.finger_l[1],normal_l,distal_l)
                finger_r = (d_r,current_node.finger_r[1],normal_r,distal_r)
            elif action == "Move down":
                z = current_node.finger_l[1] - current_node.resolution
                finger_l = (current_node.finger_l[0],z,current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (current_node.finger_r[0],z,current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Move up":
                z = current_node.finger_l[1] + current_node.resolution
                finger_l = (current_node.finger_l[0],z,current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (current_node.finger_r[0],z,current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Pivot":
                pivot_angle = current_node.pivot_angle
                left_center = find_contact_center(current_node.finger_l_poly,current_node.finger_l[2],current_node.finger_l[3],
                                                  OBJ[current_node.surf_l][0],current_node.finger_l[0],current_node.finger_l[1],
                                                  current_node.objleft_l)
                right_center = find_contact_center(current_node.finger_r_poly,current_node.finger_r[2],current_node.finger_r[3],
                                                   OBJ[current_node.surf_r][0],current_node.finger_r[0],current_node.finger_r[1],
                                                   current_node.objright_l)
                finger_poly_left = pivot_finger(pivot_angle,current_node.finger_l_poly,current_node.finger_l[2],left_center,current_node.finger_l[3])
                finger_poly_right = pivot_finger(-pivot_angle,current_node.finger_r_poly,current_node.finger_r[2],right_center,current_node.finger_r[3])
#                 finger_poly_left = pivot_finger(pivot_angle,current_node.finger_l_poly,current_node.finger_l[2],
#                                                 current_node.c_l.center_point,current_node.finger_l[3])
#                 finger_poly_right = pivot_finger(-pivot_angle,current_node.finger_r_poly,current_node.finger_r[2],
#                                                  current_node.c_r.center_point,current_node.finger_r[3])
                d_l,z = get_finger_param(finger_poly_left,OBJ)
                d_r,z = get_finger_param(finger_poly_right,OBJ)
                finger_l = (d_l,z,finger_poly_left[1],finger_poly_left[2])
                finger_r = (d_r,z,finger_poly_right[1],finger_poly_right[2])


                
            new_node = Node(finger_l,finger_r,parent=current_node,action=action)

            children.append(new_node)

        # for child in children:
        #     print(child.action)
        for child in children:
            
            if len([visited_child for visited_child in visited_list if
                   visited_child == child]) > 0:
                # print(child.action)
                continue

            
            child.g = current_node.g + cost_list[child.action]

            # Distance between each corner of the contact and goal region
            x,y = child.contact_l.exterior.xy
            a = list(zip(x[:-1],y[:-1]))
            dists = []
            for goal_key in child.goal_list_l:
                goal = child.goal_list_l[goal_key]
                sum = 0
                for point in a:
                    sum += sPoint(point).distance(goal)**2
                dists.append(sum)

            h_l = min(dists)

            # Right
            x,y = child.contact_r.exterior.xy
            a = list(zip(x[:-1],y[:-1]))
            dists = []

            for goal_key in child.goal_list_r:
                goal = child.goal_list_r[goal_key]
                sum = 0
                for point in a:
                    sum += sPoint(point).distance(goal)**2
                dists.append(sum)

            h_r = min(dists)


            child.h = (h_l + h_r)/2

            if child.action == current_node.action:
                child.h *= 0.7 #My beta: 0.5 Joshua's beta: 0.7

            child.f = child.g + child.h*100 #My epsilon: 5 Joshua's epsilon: 2


            if len([i[1] for i in yet_to_visit_list.queue if child == i[1] and
                    child.g > i[1].g]) > 0:
                continue
            
            # print("Action: {}, Cost: {}".format(child.action,child.f))
            yet_to_visit_list.put((child.f,child))


def return_path(current_node):
    path = []
    actions = []
    current = current_node
    while current is not None:
        path.append((current.finger_l_poly,current.finger_r_poly))
        actions.append((current.action,current.command))
        current = current.parent

    path = path[::-1]
    actions = actions[::-1]

    return path, actions



if __name__=="__main__":

    obj_name = "square_prism"
    exp_no = 4
    obj_type, goal_no, z, d_l, d_r, normal_l, distal_l, normal_r, distal_r = get_experimental_setup(obj_name,exp_no)
    OBJ = get_OBJ(obj_type,goal_no)
    OBJ_neighbors = get_neighbors(OBJ)
    # print(OBJ_neighbors)
    costs = {"Slide left down":1, "Slide left up":1, "Slide right down":1, "Slide right up":1,
     "Rotate ccw":3, "Rotate cw":3, "Move up":10, "Move down": 10, "Pivot":25}

    print("z=",z)
    print("dl=",d_l)
    print("dr=",d_r)
    
    # z=0
    # d_l=8
    # d_r=11.5
    # normal_l=Vector(0,1,0)
    # distal_l=Vector(1,0,0)
    # normal_r=Vector(0,-1,0)
    # distal_r=Vector(1,0,0)
    
    # normal_l=Vector(1,0,0)
    # distal_l=Vector(0,0,-1)
    # normal_r=Vector(-1,0,0)
    # distal_r=Vector(0,0,-1)

    finger_l = (d_l,z,normal_l,distal_l)
    finger_r = (d_r,z,normal_r,distal_r)

    ex = Node(finger_l,finger_r)
    print(ex.available_actions)
    ex.plot_state()


    path, actions, dt = search(ex,costs)

    file_path = obj_type + "_" + str(exp_no) + "_" + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M')
    os.mkdir(file_path)
    # param_file = open(file_path+"/Test_details.txt","w")
    # data_name = ["Palm width","Finger width","Finger length","Finger height","Object dimensions","Finger end","Finger start",
    # "start","end","time"]
    # param_file.write(str(data_name) + "\n")
    plan_file = open(file_path+"/data.txt","w")
    data_name = ["Step","command"]
    plan_file.write(str(data_name)+"\n")

    # data = [PALM_WIDTH,FINGER_WIDTH,finger_w,finger_h,(OBJECT_SIZE,OBJECT_SIZE,surf_dims[0][1]),FINGER_END,FINGER_START,(s_l,d_l,s_r,d_r,z+surf_dims[s_l][1]/2),(end_node.surface_l,end_node.d_l,end_node.surface_r,end_node.d_r,end_node.z),dt]
    # param_file.write(str(data)+"\n")
    for i in range(1,len(path)):
            
        if actions[i][0]==None:
            continue
        else:
            data = [i,actions[i]]
            
        plan_file.write(str(data)+"\n")

