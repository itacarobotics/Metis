#!/usr/bin/env python3


'''
This class is intended to calculate the inverse geometry of a given robot with the Gauss Newton method.

The key-methods for using this class are:
    + __init__ -> model, data, frame_id
    + compute_inverse_geometry: q, pos_des -> q_des
        
'''


from numpy.linalg import norm, inv
import numpy as np
import pinocchio as pin

from deltarobot import configuration as conf


class InverseGeometry():

    def __init__(self, model, data, frame_id):
        self.model = model
        self.data = data
        self.frame_id = frame_id

        # import parameters
        self.max_iterations = 300
        self.max_back_tracking_iterations = 30
        self.absolute_pos_threshold = 1e-3      # absolute tolerance on position error
        self.gradient_threshold = 1e-3          # absolute tolerance on gradient's norm
        self.beta = 0.1                         # backtracking line search parameter
        self.gamma = 1e-2                       # line search convergence parameters
        self.hessian_regu = 1e-2                # Hessian regularization
        return
    

    # Gauss-Newton algorithm
    def gauss_newton_step(self, q, pos, pos_des, J):
        e = pos_des - pos
        cost = norm(e)
        
        # Newton method
        nv = J.shape[1]
        B = J.T.dot(J) + self.hessian_regu*np.eye(nv) # approximate regularized Hessian
        gradient = J.T.dot(e)   # gradient
        delta_q = inv(B).dot(gradient)
        q_next = q + delta_q
        
        # if gradient is null you are done
        grad_norm = norm(gradient)
        if(grad_norm < self.gradient_threshold):
            # print("Terminate because gradient is (almost) zero:", grad_norm)
            # print("Problem solved after %d iterations with error %f"%(i, norm(e)))
            return None
        
        # if error is null you are done
        if(cost < self.absolute_pos_threshold):
            # print("Problem solved after %d iterations with error %f"%(i, norm(e)))
            return None
        

        # back-tracking line search
        alpha = 1.0
        iter_line_search = 0
        
        while True:
            q_next = q + alpha*delta_q

            pin.forwardKinematics(self.model, self.data, q_next)
            pos_new = pin.updateFramePlacement(self.model, self.data, self.frame_id).translation

            cost_new = norm(pos_des - pos_new)

            if cost_new < (1.0-alpha*self.gamma)*cost:
                # print("Backtracking line search converged with log(alpha)=%.1f"%np.log10(alpha))
                break
            else:
                alpha *= self.beta
                iter_line_search += 1
                if(iter_line_search==self.max_back_tracking_iterations):
                    # print("Backtracking line search could not converge. log(alpha)=%.1f"%np.log10(alpha))
                    break
        
        # print("Iteration %d, ||pos_des-x||=%f, norm(gradient)=%f"%(i, norm(e), grad_norm))
        return q_next
    
    # method computes inverse geomtery
    def compute_inverse_geometry(self, q, pos_des):
        q_next = q

        while q_next is not None:
            q = q_next

            pin.forwardKinematics(self.model, self.data, q)
            pos = pin.updateFramePlacement(self.model, self.data, self.frame_id).translation

            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id)
            J = J6[0:3,:]

            q_next = self.gauss_newton_step(q, pos, pos_des, J)

        return q





# ***********************************************************************************************
## test the algorithm
# from os.path import join

# import pinocchio as pin


# def main():
#     # import urdf file path
#     package_path = "/home/ostifede02/Documents/2dr_ws/src/deltarobot_description"
#     # chain 1
#     urdf_file_name_chain_1 = "deltarobot_c1.urdf"
#     urdf_file_path_chain_1 = join(join(package_path, "urdf"), urdf_file_name_chain_1)
#     # chain 2
#     urdf_file_name_chain_2 = "deltarobot_c2.urdf"
#     urdf_file_path_chain_2 = join(join(package_path, "urdf"), urdf_file_name_chain_2)
#     # chain 3
#     urdf_file_name_chain_3 = "deltarobot_c3.urdf"
#     urdf_file_path_chain_3 = join(join(package_path, "urdf"), urdf_file_name_chain_3)

#     # Load the urdf models
#     model_chain_1 = pin.buildModelFromUrdf(urdf_file_path_chain_1)
#     data_chain_1 = model_chain_1.createData()
#     model_chain_2 = pin.buildModelFromUrdf(urdf_file_path_chain_2)
#     data_chain_2 = model_chain_2.createData()
#     model_chain_3 = pin.buildModelFromUrdf(urdf_file_path_chain_3)
#     data_chain_3 = model_chain_3.createData()

#     # initialize InverseGeometry object
#     ig_chain_1 = InverseGeometry(model_chain_1, data_chain_1, 10)
#     ig_chain_2 = InverseGeometry(model_chain_2, data_chain_2, 10)
#     ig_chain_3 = InverseGeometry(model_chain_3, data_chain_3, 10)
    
#     q0 = np.array([0, 0, 0])
#     pos_des = np.array([0, 0, -200])

#     ee_radius = 25

#     ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*ee_radius
#     pos_next_1 = pos_des + ee_1_offset
#     q_next_1 = ig_chain_1.compute_inverse_geometry(q0, pos_next_1)

#     ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*ee_radius
#     pos_next_2 = pos_des + ee_2_offset
#     q_next_2 = ig_chain_2.compute_inverse_geometry(q0, pos_next_2)

#     ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*ee_radius
#     pos_next_3 = pos_des + ee_3_offset
#     q_next_3 = ig_chain_3.compute_inverse_geometry(q0, pos_next_3)

#     print(f"pos: {pos_des}")
#     print(f"q_1: {q_next_1}")
#     print(f"q_2: {q_next_2}")
#     print(f"q_3: {q_next_3}")
#     return


# if __name__ == "__main__":
#     main()