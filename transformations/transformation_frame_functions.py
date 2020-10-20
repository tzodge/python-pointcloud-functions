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

def plot_frame(frame, ax_handle):
    R = frame[0:3,0:3] 
    frame_orig = frame[0:3,0:3] 
    x_axis,y_axis,z_axis = R[:,0], R[:,1], R[:,2]   
    frame_orig = frame[0:3,3]
    # c = np.random.rand(3,)
    s = 0.1

    ax_handle.quiver(frame_orig[0], frame_orig[1], frame_orig[2], \
                s * x_axis[0], \
                s * x_axis[1], \
                s * x_axis[2], color='r') # plotting x_axis  
    ax_handle.quiver(frame_orig[0], frame_orig[1], frame_orig[2], \
                s * y_axis[0], \
                s * y_axis[1], \
                s * y_axis[2], color='g') # plotting x_axis  
    ax_handle.quiver(frame_orig[0], frame_orig[1], frame_orig[2], \
                s * z_axis[0], \
                s * z_axis[1], \
                s * z_axis[2], color='b') # plotting x_axis  

def plot_frames(frames,ax_handle):


    ax_handle.scatter(0, 0, 0, s=10,  marker='o')  
    ax_handle.text(0, 0, 0, 'base')   
    plot_frame(np.eye(4),ax_handle)
    for i,frame in zip(range(len(frames)),frames): 
    # for frame in frames: 
 
        plot_frame(frame,ax_handle) 
        np.set_printoptions(precision=2)

 
        # ax_handle.scatter(frame_orig[0], frame_orig[1], frame_orig[2], s=100,  marker='o')  
        ax_handle.text(frame[0,3], frame[1,3], frame[2,3], ' {}'.format(i))     
        ax_handle.set_xlabel('x_axis')
        ax_handle.set_ylabel('y_axis')
        ax_handle.set_zlabel('z_axis')

    # ax_handle.set_aspect('equal')


def main():
    fig = plt.figure()
    ax_handle = fig.add_subplot(111, projection="3d")

    test_frame = generate_transf(axis=[0,0,1], angle=np.pi/6, translation=[0.2,0,0])[0]


    frame_array = np.array([test_frame])

    plot_frames(frame_array,ax_handle)
    ax_handle.set_aspect("equal")
    ax_handle.set_xlim([-0.3,0.3])
    ax_handle.set_ylim([-0.3,0.3])
    ax_handle.set_zlim([-0.3,0.3])
    plt.show()    

if __name__ == '__main__':
    main()