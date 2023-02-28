import sys
import numpy as np
import pinocchio as pin

from flexible_pendulum import FlexiblePendulum
from simulation import simulate_flexible_pendulum
from visualization import visualize_elastic_pendulum
from controllers import DummyController, PDController


if __name__ == "__main__":
    # Define simulation parameters
    dt = 0.01
    n_sim_steps = 500

    # Instantiate the robot
    n_seg = 10
    robot = FlexiblePendulum(n_seg)

    # Specify controller
    # cntr = DummyController(1)
    cntr = PDController(Kp=2.5, Kd=0.05)

    # Specify reference output (active joint positions & velocities)
    y_ref_1 = np.zeros((int(n_sim_steps/2), 2))
    y_ref_2 = np.hstack((
        np.pi/4*np.ones((int(n_sim_steps/2)+1, 1)),
        np.zeros((int(n_sim_steps/2)+1, 1))
    ))
    y_ref = np.vstack((y_ref_1, y_ref_2))

    # Specify the initial state
    q0 = np.zeros((robot.model.nq,1))
    dq0 = np.zeros((robot.model.nq,1))
    x0 = np.vstack((q0, dq0))
    
    # Simulate the pendulum
    t, x = simulate_flexible_pendulum(robot, cntr, y_ref, x0, dt)

    # Parse states and visualize
    q, dq = np.hsplit(x, 2)

    visualize_elastic_pendulum(q, dt, n_replays=10, urdf_path=robot.urdf_path)

    