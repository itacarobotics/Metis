#!/usr/bin/env python3


import numpy as np
from numpy.linalg import norm

from deltarobot import configuration as conf


class TrajectoryGenerator():
    def init(self):
        # import parameters
        self.max_velocity_default = conf.configuration["trajectory"]["max_velocity"]
        self.max_acceleration_default = conf.configuration["trajectory"]["max_acceleration"]
        self.min_delta_time = conf.configuration["trajectory"]["min_delta_time"]
        return
    

    def generate_trajectory_joint_space(self, q_start, q_end, t_total_input):
        # get overall max task time
        t_total_1 = max(
            self.get_max_time_vel_constrained(q_start[0], q_end[0]),
            self.get_max_time_acc_constrained(q_start[0], q_end[0]))
        t_total_2 = max(
            self.get_max_time_vel_constrained(q_start[1], q_end[1]),
            self.get_max_time_acc_constrained(q_start[1], q_end[1]))
        t_total_3 = max(
            self.get_max_time_vel_constrained(q_start[2], q_end[2]),
            self.get_max_time_acc_constrained(q_start[2], q_end[2]))
        
        t_total = max(t_total_1, t_total_2, t_total_3, t_total_input)
        
        linspace_resolution = int(t_total / self.min_delta_time)+1
        t_instance = np.linspace(0, t_total, linspace_resolution)

        # joint trajectory chain 1
        a0 = q_start[0]
        # a1 = 0, a2 = 0
        a3 = -(10*(q_start[0]-q_end[0]))/pow(t_total, 3)
        a4 = (15*(q_start[0]-q_end[0]))/pow(t_total, 4)
        a5 = -(6*(q_start[0]-q_end[0]))/pow(t_total, 5)

        s1 = self.quintic_polinomial(a0, 0, 0, a3, a4, a5, t_instance)

        # joint trajectory chain 2
        a0 = q_start[1]
        # a1 = 0, a2 = 0
        a3 = -(10*(q_start[1]-q_end[1]))/pow(t_total, 3)
        a4 = (15*(q_start[1]-q_end[1]))/pow(t_total, 4)
        a5 = -(6*(q_start[1]-q_end[1]))/pow(t_total, 5)

        s2 = self.quintic_polinomial(a0, 0, 0, a3, a4, a5, t_instance)

        # joint trajectory chain 3
        a0 = q_start[2]
        # a1 = 0, a2 = 0
        a3 = -(10*(q_start[2]-q_end[2]))/pow(t_total, 3)
        a4 = (15*(q_start[2]-q_end[2]))/pow(t_total, 4)
        a5 = -(6*(q_start[2]-q_end[2]))/pow(t_total, 5)

        s3 = self.quintic_polinomial(a0, 0, 0, a3, a4, a5, t_instance)

        return np.array([s1, s2, s3, t_instance]).transpose()
    

    def get_max_time_vel_constrained(self, q_start, q_end):
        t_total = abs((15*(q_start-q_end))/(8*self.max_velocity_default))
        return t_total
    

    def get_max_time_acc_constrained(self, q_start, q_end):
        t_total = (3*np.sqrt(5/2)*np.sqrt(abs(-q_start+q_end)))/(2*np.sqrt(self.max_acceleration_default))
        return t_total
    

    def quintic_polinomial(self, a0, a1, a2, a3, a4, a5, t):
        s = a0
        s += a1*t
        s += a2*pow(t, 2)
        s += a3*pow(t, 3)
        s += a4*pow(t, 4)
        s += a5*pow(t, 5)
        return s
    
    


    def generate_trajectory_task_spacebezier(self, pos_start, pos_end, t_total_input, path_routine_type):
        
        # the cubic bezier curve is a plynomial described by 4 points
        self.path_poly_points = self.get_path_poly_points(pos_start, pos_end, path_routine_type)
                  
        x_total = self.get_path_length()

        n_set_points = self.get_number_set_points(x_total)     # avoid via points too close to each other

        # if the trajectory is time constrained (t > 0) -> set new max velocity, else default max velocity
        if t_total_input > 0:
            const_velocity_ = self.get_const_velocity(t_total_input, x_total)
            if const_velocity_ is None:
                return None
            else:
                self.const_velocity = const_velocity_
        
        else:
            self.const_velocity = self.max_velocity_default

        self.const_acceleration = self.max_acceleration_default
        
        # time scaling profile flags
        self.x_acc_flag, self.t_acc_flag, t_total = self.get_time_scaling_flags(x_total)

        # creating the set points vector
        set_points_vector = np.empty((n_set_points, 4))
        set_point = np.empty(3)
        set_point_prev = self.get_pos_bezier_poly(0)
        x_travelled = 0
        

        s_instance = np.linspace(0, 1, n_set_points)
        for index, s in enumerate(s_instance):
            set_point = self.get_pos_bezier_poly(s)
            
            x_travelled += np.linalg.norm(set_point - set_point_prev)            

            if s == 1:
                t_travelled = self.get_t_next(x_total, x_total, t_total)
            else:
                t_travelled = self.get_t_next(x_travelled, x_total, t_total)

            set_points_vector[index, 0] = set_point[0]
            set_points_vector[index, 1] = set_point[1]
            set_points_vector[index, 2] = set_point[2]
            set_points_vector[index, 3] = t_travelled

            set_point_prev = set_point

        return set_points_vector
    

    # def generate_trajectory_task_space_point_to_pointcontinuous(self, pos_start, pos_end, t_total_input):

    #     x_total = np.linalg.norm(pos_end - pos_start)

    #     n_set_points = self.get_number_set_points(x_total)     # avoid via points too close to each other

    #     # if the trajectory is time constrained (t > 0) -> set new max velocity, else default max velocity
    #     if t_total_input > 0:
    #         self.const_acceleration = self.max_acceleration_default
    #         const_velocity_ = self.get_const_velocity(t_total_input, x_total)
    #         if const_velocity_ is None:
    #             return None
    #         else:
    #             self.const_velocity = const_velocity_
    #     else:
    #         self.const_velocity = self.max_velocity_default
    #         self.const_acceleration = self.max_acceleration_default

        
    #     # time scaling profile flags
    #     self.x_acc_flag, self.t_acc_flag, t_total = self.get_time_scaling_flags(x_total)

    #     # creating the set points vector
    #     set_points_vector = np.empty((n_set_points, 4))
    #     set_point = np.empty(3)
    #     set_point_prev = pos_start
    #     x_travelled = 0
        

    #     s_instance = np.linspace(0, 1, n_set_points)
    #     for index, s in enumerate(s_instance):
    #         set_point = (1-s)*pos_start + s*pos_end
            
    #         x_travelled += np.linalg.norm(set_point - set_point_prev)            

    #         if s == 1:
    #             t_travelled = self.get_t_next(x_total, x_total, t_total)
    #         else:
    #             t_travelled = self.get_t_next(x_travelled, x_total, t_total)

    #         set_points_vector[index, 0] = set_point[0]
    #         set_points_vector[index, 1] = set_point[1]
    #         set_points_vector[index, 2] = set_point[2]
    #         set_points_vector[index, 3] = t_travelled

    #         set_point_prev = set_point

    #     return set_points_vector

    def generate_trajectory_task_space_point_to_pointcontinuous(self, pos_start, pos_end, t_total_input):
        x_total = np.linalg.norm(pos_end - pos_start)
        n_set_points = self.get_number_set_points(x_total)     # avoid via points too close to each other

        # get overall max task time
        t_total_constrained = max(
            self.get_max_time_vel_constrained(0, x_total),
            self.get_max_time_acc_constrained(0, x_total))
        
        t_total = max(t_total_constrained, t_total_input)

        set_points_vector = np.empty((n_set_points, 4))

        ## define quintic polynomial coefficients
        # a0 = 0, a1 = 0, a2 = 0
        a3 = 10/pow(t_total, 3)
        a4 = -15/pow(t_total, 4)
        a5 = 6/pow(t_total, 5)

        t_instance = np.linspace(0, t_total, n_set_points)
        s_instance = self.quintic_polinomial(0, 0, 0, a3, a4, a5, t_instance)

        for index, s in enumerate(s_instance):
            set_point = (1-s)*pos_start + s*pos_end

            set_points_vector[index, 0] = set_point[0]
            set_points_vector[index, 1] = set_point[1]
            set_points_vector[index, 2] = set_point[2]
            set_points_vector[index, 3] = t_instance[index]

        return set_points_vector


    def generate_trajectorytask_space_point_to_point_direct(self, pos_start, pos_end, t_total_input):
        set_points_vector = np.empty((2, 4))

        # get optimal trajectory time
        x_total = np.linalg.norm(pos_end - pos_start)
        delta_t_vel_constrained = x_total / self.max_velocity_default
        delta_t = max(t_total_input, delta_t_vel_constrained)
        
        ## add to values vector
        set_points_vector[0, 0] = pos_start[0]
        set_points_vector[0, 1] = pos_start[1]
        set_points_vector[0, 2] = pos_start[2]
        set_points_vector[0, 3] = 0

        set_points_vector[1, 0] = pos_end[0]
        set_points_vector[1, 1] = pos_end[1]
        set_points_vector[1, 2] = pos_end[2]
        set_points_vector[1, 3] = delta_t
        
        return set_points_vector


    def get_path_poly_points(self, pos_start, pos_end, path_routine_type):

        if path_routine_type == conf.PICK_TRAJECTORY:
            x2_offset = 0
            y2_offset = 0
            z2_offset = (pos_start[2] - pos_end[2])*0.4

            x1_offset = ((pos_end[0]+x2_offset)-pos_start[0])*0.4
            y1_offset = ((pos_end[1]+y2_offset)-pos_start[1])*0.4
            z1_offset = ((pos_end[2]+z2_offset)-pos_start[2])*0.5

        elif path_routine_type == conf.PLACE_TRAJECTORY:
            x1_offset = 0
            y1_offset = 0
            z1_offset = min(norm(pos_start[0:2] - pos_end[0:2])*0.4, 70)
            z1_offset = max(norm(pos_start[0:2] - pos_end[0:2])*0.4, 30)

            x2_offset = 0
            y2_offset = 0
            z2_offset = min(norm(pos_start[0:2] - pos_end[0:2])*0.4, 70)
            z2_offset = max(norm(pos_start[0:2] - pos_end[0:2])*0.4, 30)

        else:
            return None
        
        P1 = np.array([pos_start[0]+x1_offset, pos_start[1]+y1_offset, pos_start[2]+z1_offset])
        P2 = np.array([pos_end[0]+x2_offset, pos_end[1]+y2_offset, pos_end[2]+z2_offset])

        path_poly_points = np.array([pos_start, P1, P2, pos_end])

        return path_poly_points
    

    def get_path_length(self):
        x_total = 0
        delta_s = conf.configuration["trajectory"]["delta_s_high_resolution"]
        pos_current = self.get_pos_bezier_poly(0)

        # compute curve's length
        s_instance = np.linspace(0, 1-delta_s, int(1/delta_s))
        for s in s_instance:
            pos_next = self.get_pos_bezier_poly(s+delta_s)
            x_total += np.linalg.norm(pos_next-pos_current)
            pos_current = pos_next

        return x_total
    

    def get_pos_bezier_poly(self, s):
        pos = np.empty(3)
        pos = pow(1-s, 3)*self.path_poly_points[0]
        pos += 3*pow(1-s, 2)*s*self.path_poly_points[1] 
        pos += 3*(1-s)*pow(s, 2)*self.path_poly_points[2] 
        pos += pow(s, 3)*self.path_poly_points[3]

        return pos
    

    def get_number_set_points(self, x_total):
        min_distance_between_set_points = conf.configuration["trajectory"]["min_distance_between_set_points"]
        mean_distance_between_set_points = conf.configuration["trajectory"]["mean_distance_between_set_points"]
        
        # if the travel distance is to short -> go directly there
        if x_total <= min_distance_between_set_points:
            n_set_points = 2
        else:
            n_set_points = int(x_total/mean_distance_between_set_points)

        return n_set_points


    def get_const_velocity(self, t_total, x_total):
        # check if is physically possible
        delta = pow(t_total * self.const_acceleration, 2)-4*x_total*self.const_acceleration
        
        if delta < 0:
            return None
        
        velocity = (t_total * self.const_acceleration-np.sqrt(pow(t_total * self.const_acceleration, 2)-4*x_total*self.const_acceleration))/2
        return velocity
    

    def get_time_scaling_flags(self, x_total):
    
        x_acc_flag = pow(self.const_velocity,2) / (2*self.const_acceleration)
        
        # if there isn't a constant velocity profile
        if 2*x_acc_flag >= x_total:
            x_acc_flag = x_total * 0.5

        # time flags
        t_acc_flag = np.sqrt((2*x_acc_flag)/self.const_acceleration)
        t_total = np.sqrt((2*x_acc_flag)/self.const_acceleration)
        t_total += ((x_total-2*x_acc_flag)/self.const_velocity) 
        t_total += np.sqrt((2*abs(x_acc_flag)/self.const_acceleration))

        return x_acc_flag, t_acc_flag, t_total


    def get_t_next(self, x_travelled, x_total, t_total):
        # acceleration profile
        if x_travelled < self.x_acc_flag:
            t_next = np.sqrt((2*x_travelled)/self.const_acceleration)

        # constant velocity profile
        if x_travelled >= self.x_acc_flag and x_travelled < (x_total - self.x_acc_flag):
            t_next = self.t_acc_flag + ((x_travelled - self.x_acc_flag) / self.const_velocity)

        # deceleration profile
        if x_travelled >= (x_total - self.x_acc_flag):
            t_next = t_total - np.sqrt((2*abs(x_total - x_travelled) / self.const_acceleration))

        return t_next






# # test the algorithm
# import configuration as conf
# from trajectory_generator import TrajectoryGenerator

# import matplotlib.pyplot as plt
# import numpy as np
# import time


# def main():
#     trajectory_generator = TrajectoryGenerator()
    
#     pos_start = np.array([100, -200, -200])
#     pos_end = np.array([0, 200, -100])
#     task_time = -1


#     t_start = time.time()
#     trajectory_vector = trajectory_generator.generate_trajectory_task_space_point_to_pointcontinuous(pos_start, pos_end, task_time)
#     t_end = time.time()
#     print(f"Computed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")


#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot(trajectory_vector[:,0], trajectory_vector[:,1],trajectory_vector[:,2], marker="o")
#     # ax = fig.add_subplot(111)
#     # ax.plot(trajectory_vector, marker="o")
    

#     # Label axes
#     # ax.set_xlabel('X Axis')
#     # ax.set_ylabel('Y Axis')
#     # ax.set_zlabel('Z Axis')

#     ax.grid(True)
#     plt.show()


# if name == "main":
#     main()    
    




# # # test the algorithm - quintic profile
# # import configuration as conf
# # from trajectory_generator import TrajectoryGenerator

# # import matplotlib.pyplot as plt
# # import numpy as np
# # import time


# # def main():
# #     trajectory_generator = TrajectoryGenerator()
    
# #     # q_start = np.array([100, 120, 300])
# #     # q_end = np.array([100, 250, 50])
# #     # task_time = -1

# #     # t_start = time.time()
# #     # trajectory_vector = trajectory_generator.generate_trajectory_joint_space(
# #     #     q_start, q_end, task_time)
# #     # t_end = time.time()
# #     # print(f"Computed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")

    
# #     pos_start = np.array([0, -100, -200])
# #     pos_end = np.array([0, 100, -200])
# #     task_time = -1

# #     t_start = time.time()
# #     trajectory_vector = trajectory_generator.generate_trajectory_joint_space(
# #         pos_start, pos_end, task_time)
# #     t_end = time.time()
# #     print(f"Computed time: {round((t_end - t_start)*1e3, 3)} [ ms ]")



# #     ## PLOT POSITION
# #     plt.figure()
# #     plt.plot(trajectory_vector[0,:], trajectory_vector[1,:], trajectory_vector[2,:], 'r')
# #     # Label axes
# #     plt.xlabel('time in sec')
# #     plt.ylabel('joint position in mm')
# #     plt.grid(True)


# #     # ## PLOT VELOCITY
# #     # trajectory_vector_vel = np.empty((4, len(trajectory_vector[0])))
# #     # trajectory_vector_vel[0,:] = np.gradient(trajectory_vector[0,:],trajectory_vector[3,:])

# #     # plt.figure()
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_vel[0,:], 'r')
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_vel[1,:], 'g')
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_vel[2,:], 'b')
# #     # # Label axes
# #     # plt.xlabel('time in sec')
# #     # plt.ylabel('joint velocity in mm/s')
# #     # plt.grid(True)


# #     # ## PLOT ACCELERATION
# #     # trajectory_vector_acc = np.empty((4, len(trajectory_vector_vel[0])))
# #     # trajectory_vector_acc[0,:] = np.gradient(trajectory_vector_vel[0,:],trajectory_vector[3,:])
# #     # trajectory_vector_acc[1,:] = np.gradient(trajectory_vector_vel[1,:],trajectory_vector[3,:])
# #     # trajectory_vector_acc[2,:] = np.gradient(trajectory_vector_vel[2,:],trajectory_vector[3,:])

# #     # plt.figure()
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_acc[0,:], 'r')
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_acc[1,:], 'g')
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_acc[2,:], 'b')
# #     # # Label axes
# #     # plt.xlabel('time in sec')
# #     # plt.ylabel('joint acceleration in mm/s2')
# #     # plt.grid(True)


# #     # ## PLOT JERK
# #     # trajectory_vector_jerk = np.empty((4, len(trajectory_vector_acc[0])))
# #     # trajectory_vector_jerk[0,:] = np.gradient(trajectory_vector_acc[0,:],trajectory_vector[3,:])
# #     # trajectory_vector_jerk[1,:] = np.gradient(trajectory_vector_acc[1,:],trajectory_vector[3,:])
# #     # trajectory_vector_jerk[2,:] = np.gradient(trajectory_vector_acc[2,:],trajectory_vector[3,:])

# #     # plt.figure()
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_jerk[0,:], 'r')
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_jerk[1,:], 'g')
# #     # plt.plot(trajectory_vector[3,:], trajectory_vector_jerk[2,:], 'b')
# #     # # Label axes
# #     # plt.xlabel('time in sec')
# #     # plt.ylabel('joint jerk in mm/s3')
# #     # plt.grid(True)

# #     plt.show()




# # if name == "main":
# #     main()    












#     # ## PLOT POSITION
#     # plt.figure()
#     # plt.plot(trajectory_vector[3,:], trajectory_vector[0,:], 'r')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector[1,:], 'g')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector[2,:], 'b')
#     # # Label axes
#     # plt.xlabel('time in sec')
#     # plt.ylabel('joint position in mm')
#     # plt.grid(True)


#     # ## PLOT VELOCITY
#     # trajectory_vector_vel = np.empty((4, len(trajectory_vector[0])))
#     # trajectory_vector_vel[0,:] = np.gradient(trajectory_vector[0,:],trajectory_vector[3,:])
#     # trajectory_vector_vel[1,:] = np.gradient(trajectory_vector[1,:],trajectory_vector[3,:])
#     # trajectory_vector_vel[2,:] = np.gradient(trajectory_vector[2,:],trajectory_vector[3,:])

#     # plt.figure()
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_vel[0,:], 'r')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_vel[1,:], 'g')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_vel[2,:], 'b')
#     # # Label axes
#     # plt.xlabel('time in sec')
#     # plt.ylabel('joint velocity in mm/s')
#     # plt.grid(True)


#     # ## PLOT ACCELERATION
#     # trajectory_vector_acc = np.empty((4, len(trajectory_vector_vel[0])))
#     # trajectory_vector_acc[0,:] = np.gradient(trajectory_vector_vel[0,:],trajectory_vector[3,:])
#     # trajectory_vector_acc[1,:] = np.gradient(trajectory_vector_vel[1,:],trajectory_vector[3,:])
#     # trajectory_vector_acc[2,:] = np.gradient(trajectory_vector_vel[2,:],trajectory_vector[3,:])

#     # plt.figure()
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_acc[0,:], 'r')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_acc[1,:], 'g')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_acc[2,:], 'b')
#     # # Label axes
#     # plt.xlabel('time in sec')
#     # plt.ylabel('joint acceleration in mm/s2')
#     # plt.grid(True)


#     # ## PLOT JERK
#     # trajectory_vector_jerk = np.empty((4, len(trajectory_vector_acc[0])))
#     # trajectory_vector_jerk[0,:] = np.gradient(trajectory_vector_acc[0,:],trajectory_vector[3,:])
#     # trajectory_vector_jerk[1,:] = np.gradient(trajectory_vector_acc[1,:],trajectory_vector[3,:])
#     # trajectory_vector_jerk[2,:] = np.gradient(trajectory_vector_acc[2,:],trajectory_vector[3,:])

#     # plt.figure()
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_jerk[0,:], 'r')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_jerk[1,:], 'g')
#     # plt.plot(trajectory_vector[3,:], trajectory_vector_jerk[2,:], 'b')
#     # # Label axes
#     # plt.xlabel('time in sec')
#     # plt.ylabel('joint jerk in mm/s3')
#     # plt.grid(True)

#     # plt.show()