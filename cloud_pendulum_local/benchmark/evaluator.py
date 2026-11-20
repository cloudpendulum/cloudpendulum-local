from .controller import MotorState
from abc import ABC, abstractmethod

class Evaluator(ABC):
    """The base class for evaluators using the benchmarking interface."""

    @abstractmethod
    def evaluate(
        self,
        state: list[MotorState],
        controller_output: list[float],
        time: float,
    ):
        """Evaluate the current state of the system. Should be called once per
        control loop iteration.

        :param state: The state of the system for the current control loop
            iteration.
        :param controller_output: The controller output torques for the current
            control loop iteration.
        :param time: The time since the control loop started until the current
            control loop iteration.
        """
        pass

    @abstractmethod
    def get_score(self) -> float:
        """Get the score for the benchmark. Should be called once after the
        benchmark has finished.

        :return: The score for the benchmark.
        """
        pass

