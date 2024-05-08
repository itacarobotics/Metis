import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def bezier_curve(p0, p1, p2, t):
    # Bezier curve equation
    return np.outer((1 - t)**2, p0) + np.outer(2 * (1 - t) * t, p1) + np.outer(t**2, p2)

def main():
    # Define the coordinates of the points
    x_values = [1, 3]  # x-coordinates of the points
    y_values = [2, 5]  # y-coordinates of the points
    z_values = [3, 4]  # z-coordinates of the points

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points
    ax.plot(x_values, y_values, z_values, "b--",marker='o')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.set_title('point to point path')
    # Set the face color of the planes to white
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

    # Set axis scale to equal
    ax.set_box_aspect([1,1,1])

    # Generate points for the squiggle line using Bezier curve
    t_values = np.linspace(0, 1, 20)
    for i in range(len(t_values) - 1):
        t0, t1 = t_values[i], t_values[i+1]
        p0 = np.array([x_values[0], y_values[0], z_values[0]])
        p1 = np.array([1.1*(x_values[0] + x_values[1]) / 2, 1.1*(y_values[0] + y_values[1]) / 2, 0.95*(z_values[0] + z_values[1]) / 2])  # Control point
        p2 = np.array([x_values[1], y_values[1], z_values[1]])
        curve_points = bezier_curve(p0, p1, p2, np.linspace(t0, t1, 5))
        ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], color='orange')  # Plot the curve

    # Show the plot
    plt.legend(("desired path", "real path"))

    plt.show()


if __name__ == "__main__":
    main()
