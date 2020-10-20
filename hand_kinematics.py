import numpy as np
import scipy.optimize as opt
from math import*
from finger_params import*

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
