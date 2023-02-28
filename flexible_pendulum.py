import os
import yaml
import numpy as np
import pinocchio as pin


from utils import n_seg_int2str


class FlexiblePendulum:
    """ Implements flexible pendulum as an example to showcase
    Modified Rigid Finite Element Method for modeling flexible 
    and deformable objects
    """
    def __init__(self, n_seg: int) -> None:
        # Build urdf path
        model_folder = 'models/' + \
                       n_seg_int2str[n_seg] + '_segments/'
        urdf_file = 'pendulum_' + str(n_seg) + 's.urdf'
        self.urdf_path = os.path.join(model_folder, urdf_file)

        # Try to load model from urdf file
        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
        except ValueError:
            print(f"URDF file doesn't exist. Make sure path is correct!")

        # Create data required for the algorithms
        self.data = self.model.createData()

        # Load flexibility params
        params_file = 'flexibility_params.yml'
        params_path = os.path.join(model_folder, params_file)

        with open(params_path) as f:
            flexibility_params = yaml.safe_load(f)

        self.K = np.diag(flexibility_params['K'])
        self.D = np.diag(flexibility_params['D'])

        # Control jacobian matrix
        self.B = np.vstack(([1.], np.zeros((self.model.nq-1,1))))

        # Convenient variables
        self.nq = self.model.nq
        self.nx = 2*self.nq
        self.ny = 2 # active joint position and velocity
        self.nu = 1

        # Output jacobian matrreix
        self.C = np.zeros((self.ny, self.nx))
        self.C[0,0] = 1.
        self.C[1, self.nq] = 1.


    def elasticity_torques(self, q: np.ndarray, dq: np.ndarray):
        """ Computes torques due to elastic elements in the
        joints: spring-damper elements
        NOTE the first joint -- active joint -- doesn't
             have spring-damper element in it

        :param q: joint positions
        :param dq: joint velocities

        :return: torques 
        """
        return -self.K @ q - self.D @ dq 

    def forward_dynamics(self, q:np.ndarray, dq:np.ndarray, tau_a:np.ndarray):
        """ Computes forward dynamics

        :param q: joint positions
        :param dq: joint velocities
        :param tau_a: active joints torque

        :return: ddq -- aceelerations of all joints, active and passive 
        """
        tau = self.B @ tau_a + self.elasticity_torques(q, dq)

        return pin.aba(self.model, self.data, q, dq, tau).reshape(-1, 1)

    def ode(self, x: np.ndarray, u: np.ndarray):
        """ Computes ode of the robot

        :param x: robot state
        :param u: active joint torque

        :return: x_dot
        """
        q, dq = np.split(x, 2)
        return np.vstack((dq, self.forward_dynamics(q, dq, u)))

    def output_map(self, x):
        """ Computes outputs from states
        """
        return self.C @ x
    

if __name__ == "__main__":
    n_seg = 10
    robot = FlexiblePendulum(n_seg)

    q = np.random.uniform(low=-1, high=1, size=(robot.model.nq,1))
    dq = np.random.uniform(low=-1, high=1, size=(robot.model.nq,1))
    x = np.vstack((q, dq))
    tau = np.array([[1.]])
    robot.ode(x, tau)   