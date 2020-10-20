import helper 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import transforms3d as t3d

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

def plot_frame_array(frames,ax_handle):


    ax_handle.scatter(0, 0, 0, s=10,  marker='o')  
    ax_handle.text(0, 0, 0, '____base')   
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

def single_frame_plot_example():
    test_frame = helper.generate_transf(axis=[0,0,1], angle=np.pi/6, translation=[0.2,0,0])[0]
    frame_array = np.array([test_frame])



    fig = plt.figure()
    ax_handle = fig.add_subplot(111, projection="3d")
    plot_frame_array(frame_array,ax_handle)
    ax_handle.set_aspect("equal")
    ax_handle.set_xlim([-0.3,0.3])
    ax_handle.set_ylim([-0.3,0.3])
    ax_handle.set_zlim([-0.3,0.3])
    plt.show()    

def multiple_frames_plot_example():
    test_frame = helper.generate_transf(axis=[0,0,1], angle=np.pi/6, translation=[0.2,0,0])[0]

    frame_list = []
    for i in range(10):
        test_frame = helper.generate_transf(axis=[1,0,0], angle=i*np.pi/10, translation=[0.1+0.1*i,0.1+0.1*i,0])[0]
        frame_list.append(test_frame)

    frame_array = np.array(frame_list)

    fig = plt.figure()
    ax_handle = fig.add_subplot(111, projection="3d")
    plot_frame_array(frame_array,ax_handle)
    ax_handle.set_aspect("equal")
    ax_handle.set_xlim([-0.3,0.3])
    ax_handle.set_ylim([-0.3,0.3])
    ax_handle.set_zlim([-0.3,0.3])
    plt.show()    



if __name__ == '__main__':
    # single_frame_plot_example()
    multiple_frames_plot_example()