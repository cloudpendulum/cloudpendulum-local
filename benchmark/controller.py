from dataclasses import dataclass
from abc import ABC, abstractmethod
import struct

@dataclass
class MotorState:
    position: float
    velocity: float

class ControllerBase(ABC):
    """The base class for controllers using the benchmarking interface."""
    
    @abstractmethod
    def __init__(self):
        """Initialize the controller"""
        pass

    @abstractmethod
    def get_control_output(
        self,
        state: list[MotorState],
    ) -> list[float]:
        """Get the control output from the controller in the form of a list of
        torques to be applied to the actuators in the cell.

        :param state: The state of the system in the form of a list of
            MotorStates, each corresponding to an actuator in the cell running
            the experiment. For example, if you were running a DoublePendulum
            experiment the state parameter would contain two elements; one for
            the shoulder, and one for the elbow actuator.

        :return: A list of torques which should be applied to the actuators.
            For a DoublePendulum experiment this list should contain two
            elements, but the length of this list does not necessarily have to
            match the length of the state parameter. For example, the Pendbot
            and Acrobot experiments expects only one torque to be returned from
            this function. 
        """
        pass
