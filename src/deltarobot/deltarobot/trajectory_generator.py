#!/usr/bin/env python3

import numpy as np
from numpy.linalg import norm


class TrajectoryGenerator():
    
    def __init__(self, max_vel, max_acc, via_points_distance, via_points_threshold):
        # define parameters
        self.max_vel                = max_vel               # [ mm/s ]
        self.max_acc                = max_acc               # [ mm/s2 ]
        self.via_points_distance    = via_points_distance   # [ mm ]
        self.via_points_threshold   = via_points_threshold  # [ mm ]
        
        return
    
    '''

        Calculate the trajectory ...
    
    '''
    def task_space__ptp(self, pos_start, pos_end, T):
        path_length = norm(pos_end - pos_start)
        
        T_best_effort = self.get_best_effort_time(path_length)
        T = max(T_best_effort, T)

        # if T == 0:
        #     return None

        # initialize array
        n_via_points = self.get_number_via_points(path_length)
        via_points_vector = np.empty((n_via_points, 4))

        ## define quintic polynomial coefficients
        # a0 = 0, a1 = 0, a2 = 0
        a3 = 10/pow(T, 3)
        a4 = -15/pow(T, 4)
        a5 = 6/pow(T, 5)

        t_instance = np.linspace(0, T, n_via_points)
        s_instance = self.quintic_polinomial(0, 0, 0, a3, a4, a5, t_instance)

        for index, s in enumerate(s_instance):
            # parametrization of a straight line from pos_start to pos_end
            set_point = (1-s)*pos_start + s*pos_end

            # filling up datas
            via_points_vector[index, 0] = set_point[0]
            via_points_vector[index, 1] = set_point[1]
            via_points_vector[index, 2] = set_point[2]
            via_points_vector[index, 3] = t_instance[index]

        return via_points_vector


    def get_feasible_time_vel_constrained(self, q_0, q_f):
        T = abs((15*(q_0-q_f))/(8*self.max_vel))
        return T
    

    def get_feasible_time_acc_constrained(self, q_0, q_f):
        T = (np.sqrt(10)*np.sqrt(q_f-q_0)) / (pow(3, 0.25)*np.sqrt(self.max_acc))
        return T
    
    def is_time_feasable(self, T, path_length):
        # get T that does not exceed velocity and acceleration limits
        T_constrained = max(
            self.get_feasible_time_vel_constrained(0, path_length),
            self.get_feasible_time_acc_constrained(0, path_length))
        
        ## time not feasable
        if T < T_constrained:
            return False
        
        return True
    
    def get_best_effort_time(self, path_length):
        # get T that does not exceed velocity and acceleration limits
        T = max(
            self.get_feasible_time_vel_constrained(0, path_length),
            self.get_feasible_time_acc_constrained(0, path_length))
        
        return T
    

    def quintic_polinomial(self, a0, a1, a2, a3, a4, a5, t):
        s = a0
        s += a1*t
        s += a2*pow(t, 2)
        s += a3*pow(t, 3)
        s += a4*pow(t, 4)
        s += a5*pow(t, 5)
        return s
    

    def get_number_via_points(self, path_length):
        
        # if the travel distance is to short -> go directly there
        if path_length <= self.via_points_threshold:
            n_via_points = 2
        else:
            n_via_points = int(path_length/self.via_points_distance)
        
        return n_via_points








###################################################################################################
#                                                                                                 #
#                                       test the algorithm                                        #
#                                                                                                 #
###################################################################################################

# import matplotlib.pyplot as plt
# import time


# def main():
#     trajectory_generator = TrajectoryGenerator(
#         max_vel                 = 250,
#         max_acc                 = 100,
#         via_points_distance     = 15,
#         via_points_threshold    = 0
#     )
    
#     pos_start = np.array([100, -200, -200])
#     pos_end = np.array([0, 200, -100])
#     T = -1


#     t_start = time.time()
#     trajectory_vector = trajectory_generator.task_space__ptp(pos_start, pos_end, T)
#     t_end = time.time()
#     print(f"Computed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")


#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot(trajectory_vector[:,0], trajectory_vector[:,1],trajectory_vector[:,2], marker="o")

#     ax.grid(True)
    
#     plt.show()


# if __name__ == "__main__":
#     main()