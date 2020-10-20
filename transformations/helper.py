
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import transforms3d as t3d

def transformation_from_R_t(R,t):
    M = np.eye(4)
    M[0:3,0:3] = R
    M[0:3,3] = t
    return M


def generate_transf(axis=None, angle=None, translation=None):
    '''
    Generates a random transformation if no inputs are given
    Outputs: Transf matrix, axis, angle 
    '''

    if axis == None:
        axis = np.random.rand(3,) - np.array([0.5,0.5,0.5])
        axis = axis/np.linalg.norm(axis)
    if angle == None:
        angle = 2*(np.random.uniform()-0.5) * np.pi  
    if translation == None:
        translation = np.random.rand(3,)

    R = t3d.axangles.axangle2mat(axis, angle)     

    M = transformation_from_R_t(R,translation)

    return M, axis, angle

def generate_n_rand_transf(n, method="random"):
    Transf_list = []

    for i in range(n):
        if method =="random":
            Transf_list.append(generate_transf()[0])
        elif method =="structured":
            axis = [1,0,0]
            # angle = 1/180*np.pi * 10*i 
            angle = np.random.rand() 
            translation = [0.0,0,0]
            transf = generate_transf(axis=axis,
                                         angle=angle,
                                         translation=translation)[0]
            Transf_list.append(transf)

    Transf_array = np.array(Transf_list)


    return  Transf_array