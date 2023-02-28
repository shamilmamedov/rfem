import numpy as np
import yaml
import os
from typing import List

from utils import n_seg_int2str


def inertial_params_cylinder(d: float, l: float, rho: float):
    """ Computes inertial parameters of a cylindrical beam

    :param d: diameter
    :param l: length
    :param rho: density

    :return: (m, rc, I) tuple containing mass, vector
             to the center of mass and inertia tensor
    """
    # Mass
    V = np.pi*d**2*l/4
    m = rho*V

    # Center of mass
    rc = l/2

    # Second moments of inertia
    Ixx = m*d**2/8
    Iyy = m/48*(3*d**2 + 4*l**2)
    Izz = m/48*(3*d**2 + 4*l**2)

    return m, rc, np.array([Ixx, Iyy, Izz])


def spring_params_cylinder(d: float, l: float, E: float, G: float):
    """ Equivalent spring parameters of the cylindtical beam

    :param d: diameter
    :param l: length
    :param E: Young modulus
    :param G: Shear modulus

    :return: array of stiffness parameters
    """
    r = d/2

    # Get moment area of inertia
    t_ = np.pi*r**4
    Jxx, Jyy, Jzz = t_/2, t_/4, t_/4

    # Get stiffness
    kx, ky, kz = G*Jxx/l, E*Jyy/l, E*Jzz/l
    return np.array([kx, ky, kz])


def update_spring_damper_params(
    n_seg: int, 
    sde_k: List[np.ndarray], 
    sde_d: List[np.ndarray]
) -> None:
    """ Updates spring-damper parameters of the model that
    is contained in a separate .yaml file
    NOTE we are only intereseted in deformations abot Z-axis

    :param n_seg: number of segments
    :param sde_k: 
    """
    model_folder = 'models/' + \
                    n_seg_int2str[n_seg] + '_segments/'
    params_file = 'flexibility_params.yml'
    params_path = os.path.join(model_folder, params_file)

    K = [0.] + [x[2].item() for x in sde_k]
    D = [0.] + [x[2].item() for x in sde_d]
    flex_params = {'K': K, 'D': D}
    yaml_result = yaml.dump(flex_params, default_flow_style=None)
    print(yaml_result)

    with open(params_path, 'w') as stream:
        try:
            stream.write(yaml_result)
        except yaml.YAMLError as exc:
            print(exc)


if __name__ == "__main__":
    # Parameters of the rod    
    d = 0.006
    L = 1.2
    rho = 2710
    E = 7e10/3
    G = 2.7e10/3
    nu_bar = 1e9
    nu = 1e9
    zeta = 5e-3

    # Number of segments to divide
    n_seg = 10

    # Get rigid body approximation of the whole rod
    m, rc, I = inertial_params_cylinder(d, L, rho)

    # Divide into RFEs and compute inertial parameters
    rfe_lengths = [L/(2*n_seg)] + [L/n_seg]*(n_seg-1) + [L/(2*n_seg)] 

    rfe_inertial_params = [inertial_params_cylinder(d, x, rho) for x in rfe_lengths]
    rfe_m = [x[0] for x in rfe_inertial_params]
    rfe_rc = [x[1] for x in rfe_inertial_params]
    rfe_I = [x[2] for x in rfe_inertial_params]

    for k, (mk, rck, Ik, lk) in enumerate(zip(rfe_m, rfe_rc, rfe_I, rfe_lengths)):
        print(f"RFE{[k+1]}")
        print(f"\tlength = {lk:.5f}")
        print(f"\tmass = {mk:.5f}")
        print(f"\tCOM = {rck:.5f}")
        print(f"\tmoment of inertia = {Ik}")

    # Compute spring-damper parameters
    delta_l = L/n_seg
    m_delta_l = inertial_params_cylinder(d, delta_l, rho)[0]

    sde_k = [spring_params_cylinder(d, delta_l, E, G)]*n_seg

    # Compute damping using Kelvin–Voigt rheological model 
    sde_d1 = [spring_params_cylinder(d, delta_l, nu, nu_bar)]*n_seg
    
    # Compute damping using simpler method using natural frequency
    delta_l_wn = [np.sqrt(x/(m_delta_l*delta_l**2/4)) for x in sde_k]
    sde_d2 = [2*zeta*x for x in delta_l_wn]

    # Update model params
    update_model_params = True
    if update_model_params:
        update_spring_damper_params(n_seg, sde_k, sde_d1)