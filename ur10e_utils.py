
#import robot/gripper controllers
import urx
from pyrobotiqur import RobotiqGripper
#import helpers
import time
import numpy as np
import copy
import matplotlib.pyplot as plt
import pickle

HOME = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])

def T_inv(T):
    R = T[0:3, 0:3]
    d = T[0:3, 3]
    T_inv = np.vstack((np.hstack((R.T, np.expand_dims(-R.T@d, axis=1))), np.array([[0.0, 0.0, 0.0, 1.0]])))
    return T_inv

def R_from_RPY(r, p, y):
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(r), -np.sin(r)], [0.0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0.0, np.sin(p)], [0.0, 1.0, 0.0], [-np.sin(p), 0.0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0.0], [np.sin(y), np.cos(y), 0.0], [0.0, 0.0, 1.0]])
    return Rx@Ry@Rz

def T_from_DH(a, d, alpha, theta):
    return np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)], [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)], [0.0, np.sin(alpha), np.cos(alpha), d], [0.0, 0.0, 0.0, 1.0]])

def T_from_DH_modified(a, d, alpha, theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0.0, a], [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)], [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)], [0.0, 0.0, 0.0, 1.0]])
    

def UR10e_DH_table():
    # Using the convention a, d, alpha, theta
    a = [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0] #Starting at a0, alpha0, d0
    alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
    d = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655]

    return np.array([a, d, alpha]).T

def UR10e_DH_modified_table():
    a = [0.0, 0.0, 0.6127, 0.57155, 0.0, 0.0]
    alpha = [0.0, np.pi/2, 0.0, 0.0, -np.pi/2, np.pi/2]
    d = [0.0, 0.0, 0.0, 0.17415, 0.11985, 0.0]
    theta = [0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
    
    return np.array([a, d, alpha, theta]).T

def theta_modified_to_orig(thetas):
    return thetas + np.expand_dims([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0], axis=1)

def FK(thetas, T_b0=np.eye(4), T_6t = np.eye(4)):
    DH_table = UR10e_DH_table()
    T = np.eye(4)
    for i in range(0, 6):
        theta = thetas[i]
        a, d, alpha = DH_table[i, :]
        Ti = T_from_DH(a, d, alpha, theta)
        T = T@Ti
    return T_b0@T@T_6t

def safety_filter(thetas, pre_frames, post_frames):
    DH_table = UR10e_DH_table()
    T = np.eye(4)
    for frame in pre_frames:
        T = T@frame
        if T[2, 3] < -1e-6:
            return False
    for i in range(0, 6):
        theta = thetas[i]
        a, d, alpha = DH_table[i, :]
        Ti = T_from_DH(a, d, alpha, theta)
        T = T@Ti
        if T[2, 3] < -1e-6:
            return False
    for frame in post_frames:
        T = T@frame
        if T[2, 3] < -1e-6:
            return False
    return True
    

def FK_modified(thetas, T_b0 = np.eye(4), T_6t = np.eye(4)):
    DH_table_modified = UR10e_DH_modified_table()
    T = np.eye(4)
    for i in range(0, 6):
        theta = thetas[i] + DH_table_modified[i, 3]
        a, d, alpha = DH_table_modified[i, 0:3]
        Ti = T_from_DH_modified(a, d, alpha, theta)
        T = T@Ti
    return T_b0@T@T_6t

def tan_half_angle(E, F, G):
    EPSILON=1e-12
    internal = E**2+F**2-G**2
    if internal < EPSILON:
        internal = EPSILON
    t_plus, t_minus = (-F + np.sqrt(internal))/(G-E), (-F - np.sqrt(internal))/(G-E)
    return 2*np.arctan(t_plus), 2*np.arctan(t_minus)

def IK(T_bt, T_flange_t = np.eye(4), thetas_prior = HOME):
    T_b0 = np.eye(4)
    T_b0[2, 3] = 0.1807
    T_6_flange = np.eye(4)
    T_6_flange[2, 3] = 0.11655 # This is the extra offset from the 6th frame to the center of the flange in the Williams convention  

    thetas_modified = _IK(T_bt, T_b0, T_6_flange@T_flange_t)
    thetas_classical = theta_modified_to_orig(thetas_modified)
    thetas_safe_inds = [safety_filter(thetas_classical[:, i], [], [T_flange_t]) for i in range(0, thetas_classical.shape[1])]
    if not np.any(thetas_safe_inds):
        print("No safe configurations to solve IK for ", T_bt)
        return False
    thetas_classical = thetas_classical[:, thetas_safe_inds]
    nearest_theta_ind = np.argmin(np.linalg.norm(thetas_classical - np.expand_dims(thetas_prior, axis=1), axis=0)) # choose closest angle solution
    
    return thetas_classical[:, nearest_theta_ind]

def _IK(T_bt, T_b0 = np.eye(4), T_6t = np.eye(4)):
    DH_table = UR10e_DH_table()
    a, d, alpha = DH_table[:, 0], DH_table[:, 1], DH_table[:, 2]
    T_06 = T_inv(T_b0)@T_bt@T_inv(T_6t)
    thetas = np.zeros((6, 4))

    x = T_06[0, -1]
    y = T_06[1, -1]
    z = T_06[2, -1]

    r11 = T_06[0, 0]
    r12 = T_06[0, 1]
    r13 = T_06[0, 2]
    r21 = T_06[1, 0]
    r22 = T_06[1, 1]
    r23 = T_06[1, 2]
    r31 = T_06[2, 0]
    r32 = T_06[2, 1]
    r33 = T_06[2, 2]

    # theta 1
    theta1_plus, theta1_minus = tan_half_angle(y, -x, d[3])
    thetas[0, 0] = theta1_plus
    thetas[0, 1] = theta1_minus
    thetas[0, 2] = theta1_plus
    thetas[0, 3] = theta1_minus

    
    for i in range(0, 4):
        # theta 6
        theta1 = thetas[0, i]
        thetas[5, i] = np.arctan2(r12*np.sin(theta1)-r22*np.cos(theta1), r21*np.cos(theta1)-r11*np.sin(theta1))

        # theta 5
        theta6 = thetas[5, i]
        thetas[4, i] = np.arctan2((r21*np.cos(theta1) - r11*np.sin(theta1))*np.cos(theta6) + (r12*np.sin(theta1) - r22*np.cos(theta1))*np.sin(theta6), r13*np.sin(theta1) - r23*np.cos(theta1))

        # theta 2
        theta5 = thetas[4, i]

        A = (r31*np.cos(theta6)-r32*np.sin(theta6))/np.cos(theta5) 
        B = r32*np.cos(theta6) + r31*np.sin(theta6) #1

        small_a = -x*np.cos(theta1) - y*np.sin(theta1) - d[4]*A #0
        small_b = z - d[4]*B 

        E2 = 2*a[1]*small_b 
        F2 = 2*a[1]*small_a 
        G2 = a[1]**2 + small_a**2 + small_b**2 - a[2]**2 

        theta2_plus, theta2_minus = tan_half_angle(E2, F2, G2)
        if i == 0 or i == 1:
            thetas[1, i] = theta2_plus
        else:
            thetas[1, i] = theta2_minus

        # theta 3
        theta2 = thetas[1, i]
        thetas[2, i] = np.arctan2(small_a+a[1]*np.sin(theta2), small_b+a[1]*np.cos(theta2)) - theta2 # flipped a sign here

        # theta 4
        theta3 = thetas[2, i]
        thetas[3, i] = np.arctan2(A, B) - theta2 - theta3

    return thetas
