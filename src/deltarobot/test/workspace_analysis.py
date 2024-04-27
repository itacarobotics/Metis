import numpy as np
from numpy import sqrt, dot, cross                       
from numpy.linalg import norm
import matplotlib.pyplot as plt

PI = np.pi


### set parameters
'''
### IGUS deltarobot
 - carriage stroke: 146 mm
 - carriage stroke start: 170 mm
 - link length: 400 mm
'''
# define carriage stroke
carriage_stroke = 250
carriage_stroke_start = 140
# define link lengths
link_length = 400

def main():
    # define rails vectors
    rail_1_versor = np.array([1, 0, 1])*np.cos(PI/4)
    rail_1_versor = rail_1_versor / norm(rail_1_versor)

    rail_2_versor = np.array([-np.sin(PI/6), -np.cos(PI/6), 1])*np.cos(PI/4)
    rail_2_versor = rail_2_versor / norm(rail_2_versor)

    rail_3_versor = np.array([-np.sin(PI/6), np.cos(PI/6), 1])*np.cos(PI/4)
    rail_3_versor = rail_3_versor / norm(rail_3_versor)

    print(rail_1_versor)
    print(rail_2_versor)
    print(rail_3_versor)

    carriage_stroke_end = carriage_stroke_start + carriage_stroke


    # define simulation variables
    carriage_step = 6       # each carriage moves every 1 mm
    n_carriage_steps = int(carriage_stroke / carriage_step)


    # define all possible carriage positions
    carriage_1_steps = np.linspace(carriage_stroke_start, carriage_stroke_end, n_carriage_steps).reshape(-1, 1)
    carriage_1_steps = carriage_1_steps * rail_1_versor
    
    carriage_2_steps = np.linspace(carriage_stroke_start, carriage_stroke_end, n_carriage_steps).reshape(-1, 1)
    carriage_2_steps = carriage_2_steps * rail_2_versor
    
    carriage_3_steps = np.linspace(carriage_stroke_start, carriage_stroke_end, n_carriage_steps).reshape(-1, 1)
    carriage_3_steps = carriage_3_steps * rail_3_versor

    # define plot datas
    ws_points = np.empty((n_carriage_steps**3, 3))
    index = 0
    
    # get all combinations
    for C1 in carriage_1_steps:
        for C2 in carriage_2_steps:
            for C3 in carriage_3_steps:
                # compute forward kinematics
                P = three_spheres_intersection(C1, C2, C3, link_length)
                ws_points[index, :] = P
                index += 1
        print(f"[{int(100*index/n_carriage_steps**3)} %] completed")


    ## plot data
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points
    ax.scatter(ws_points[:, 0], ws_points[:, 1], ws_points[:, 2], 
               ('g', 'o'),
               edgecolors='k', 
               alpha=1)
    
    # Plot the rails
    ax.plot(carriage_1_steps[:, 0], carriage_1_steps[:, 1], carriage_1_steps[:, 2], "r", linestyle='-', linewidth=5)
    ax.plot(carriage_2_steps[:, 0], carriage_2_steps[:, 1], carriage_2_steps[:, 2], "g", linestyle='-', linewidth=5)
    ax.plot(carriage_3_steps[:, 0], carriage_3_steps[:, 1], carriage_3_steps[:, 2], "b", linestyle='-', linewidth=5)

    # Label axes
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # Show the plot
    ax.view_init(elev=15, azim=30)
    ax.set_box_aspect([1,1,1])
    plt.show()

    return



# Find the intersection of three spheres
# C1,C2,C3 are the centers, link_length,link_length,link_length are the radii       
def three_spheres_intersection(C1, C2, C3, link_length):                      
    temC1 = C2-C1                                        
    e_x = temC1/norm(temC1)                              
    temC2 = C3-C1                                        
    i = dot(e_x,temC2)                                   
    temC3 = temC2 - i*e_x                                
    e_y = temC3/norm(temC3)                              
    e_z = cross(e_x,e_y)                                 
    d = norm(C2-C1)                                      
    j = dot(e_y,temC2)                                   
    x = d*d / (2*d)
    y = (-2*i*x + i*i + j*j) / (2*j)       
    temp4 = link_length*link_length - x*x - y*y
    if temp4<0:
        raise Exception("The three spheres do not intersect!")
    z = sqrt(temp4)

    p_12_a = C1 + x*e_x + y*e_y + z*e_z
    p_12_b = C1 + x*e_x + y*e_y - z*e_z
    if p_12_a[2] < p_12_b[2]:
        P = p_12_a
    else:
        P = p_12_b

    return P



if __name__ == "__main__":
    main()


