from visualiztion_tools import plot_frame, plot_frame_array
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import transforms3d as t3d
import helper


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
    single_frame_plot_example()
    multiple_frames_plot_example()