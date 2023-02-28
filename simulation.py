import casadi as cs
import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass
from typing import Callable, Tuple

from flexible_pendulum import FlexiblePendulum


def simulate_flexible_pendulum(
    robot: FlexiblePendulum,
    controller,
    y_ref: np.ndarray,
    x0: np.ndarray,
    dt: float,
    method: str = 'RK45',
    rtol: float = 1e-6,
    atol: float = 1e-6
) -> Tuple[np.ndarray, np.ndarray]:
    """ Simulates flexible pendulum using scipy solve_ivp
    
    :param robot: FlexiblePendulum instance
    :param controller: active joint controller
    :param qa_ref: reference positions for the active joint
    :param x0: initia state of the flexible pendulum
    :param u: sequence of control inputs, which also defines the 
              number of simulation steps
    :param dt: stepsize of the integrator
    :param method: integration method
    :param rtol: relative tolerance of the integrator
    :param atol: absolute tolerance of th integrator

    :return: (t, x) a tuple of time stamps and states
    """
    def simulation_step(x, u):
        sol = solve_ivp(f, [0, dt], x, args=(u, robot),
                        vectorized=True, method=method, rtol=rtol, atol=atol)
        return sol.y[:, -1]

    # Create a lambda function that matches scipy signature
    f = lambda t, x, u, robot: robot.ode(x, u)

    n_sim_steps = y_ref.shape[0]
    output = [x0.flatten()]
    xk = x0.flatten()
    for y_ref_k in y_ref:
        yk = robot.output_map(xk)
        uk = controller.compute_inputs(yk[:,None], y_ref_k[:,None])
        xk = simulation_step(xk, uk)
        output.append(xk)

    x = np.stack(output)
    t = np.arange(n_sim_steps + 1)*dt
    return t, x



if __name__ == "__main__":
    print("DEBUGGG")
