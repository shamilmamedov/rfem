import numpy as np
from abc import ABC, abstractmethod


class BaseController(ABC):
    """ Abstract base class for controllers
    All the controllers must implement compute_torques 
    attrubute
    """

    @abstractmethod
    def compute_inputs(self, y: np.ndarray, y_ref: np.ndarray):
        """
        :param y: controlled output (controlled variables)
        :param y_ref: refernce for outputs
        """
        ...


class DummyController(BaseController):
    """ Controller that always returns zero torque
    """
    def __init__(self, n_u: int = 1) -> None:
        self.n_u = n_u

    def compute_inputs(self, y, y_ref):
        return np.zeros((self.n_u, 1))


class PDController(BaseController):
    """ Proportional-Derivative controller
    """
    def __init__(self, Kp, Kd) -> None:
        """
        :parameter Kp: proportional gain
        :parameter Kd: derivative gain
        :parmater q_ref: reference joint position
        """
        self.Kp = Kp
        self.Kd = Kd

    def compute_inputs(self, y, y_ref):
        qa, dqa = np.split(y, 2)
        qa_ref, dqa_ref = np.split(y_ref, 2)
        
        return self.Kp * (qa_ref - qa) - self.Kd * dqa



