import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer
import numpy as np
import time

from deltarobot.inverse_geometry import InverseGeometry

from os.path import join, abspath, dirname


# Load the URDF model.
urdf_file_name = "deltarobot_c1.urdf"

package_path = join(dirname(str(abspath(__file__))),"deltarobot_description")
urdf_file_path = join(join(package_path, "urdf"), urdf_file_name)
mesh_dir = join(package_path, "meshes")

# Initialize the model
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
robot = pin.RobotWrapper(model, collision_model, visual_model)

# Initialize the viewer
viz = GepettoVisualizer(model, collision_model, visual_model)
viz.initViewer(loadModel=True)
viz.loadViewerModel("pinocchio")



ig = InverseGeometry(robot)

ee_radius = 0
R = 150
Z = -250

t_instance = np.linspace(0, 2*np.pi, 50)
q = np.zeros(robot.nq)

while True:

    for t in t_instance:
        pos_des = np.array([R*np.cos(t), R*np.sin(t), Z])
        pos_des = np.array([0, 0, -400])

        ee_offset = np.array([np.sin(0), np.cos(0), 0])*ee_radius
        pos_next = pos_des + ee_offset

        q = ig.compute_inverse_geometry(q, pos_next, 10)
        print(q)
        viz.display(q)
        time.sleep(5)
