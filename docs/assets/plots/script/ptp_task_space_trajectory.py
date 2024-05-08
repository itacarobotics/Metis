import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from deltarobot.trajectory_generator import TrajectoryGenerator

def main():
    trajectory_generator = TrajectoryGenerator(
        max_vel                 = 250,
        max_acc                 = 100,
        via_points_distance     = 0.18,
        via_points_threshold    = 0
    )
    
    pos_start = np.array([1.0, 2.0, 3.0])
    pos_end = np.array([3.0, 5.0, 4.0])
    T = 8

    ## generate path
    n_via_points = 18
    s = np.linspace(0, 1, n_via_points)
    path_vector = np.empty((n_via_points, 3))

    for i in range(n_via_points):
        path_vector[i,:] = (1-s[i])*pos_start + s[i]*pos_end

    ## generate trajectory
    trajectory_vector = trajectory_generator.task_space__ptp(pos_start, pos_end, T)

    fig = plt.figure()

    # # Plot for path_vector
    # ax1 = fig.add_subplot(121, projection='3d')
    # ax1.scatter(path_vector[:,0], path_vector[:,1], path_vector[:,2], marker="o", color='blue')
    # ax1.set_xlabel('X')
    # ax1.set_ylabel('Y')
    # ax1.set_zlabel('Z')
    # ax1.set_title('via-points before interpolation')

    # ax1.set_xticks(np.linspace(min(path_vector[:,0]), max(path_vector[:,0]), 5))
    # ax1.set_yticks(np.linspace(min(path_vector[:,1]), max(path_vector[:,1]), 5))
    # ax1.set_zticks(np.linspace(min(path_vector[:,2]), max(path_vector[:,2]), 5))
    # ax1.grid(True)
    # # Set the face color of the planes to white
    # ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))


    # Plot for trajectory_vector
    ax2 = fig.add_subplot(111, projection='3d')
    ax2.plot(trajectory_vector[:,0], trajectory_vector[:,1], trajectory_vector[:,2], "--", marker="o", color='orange', label = "via points")
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    plt.legend(loc="best")
    # ax2.set_title('via-points after interpolation')
    
    ax2.set_xticks(np.linspace(min(trajectory_vector[:,0]), max(trajectory_vector[:,0]), 5))
    ax2.set_yticks(np.linspace(min(trajectory_vector[:,1]), max(trajectory_vector[:,1]), 5))
    ax2.set_zticks(np.linspace(min(trajectory_vector[:,2]), max(trajectory_vector[:,2]), 5))
    ax2.grid(True)
    # Set the face color of the planes to white
    ax2.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax2.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax2.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))    
    

    # Set axis scale to equal
    # ax1.set_box_aspect([1,1,1])
    ax2.set_box_aspect([1,1,1])
    
    plt.show()


if __name__ == "__main__":
    main()
