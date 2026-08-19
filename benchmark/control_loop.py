from .controller import MotorState
from ..client import Client
from ..disturbance import Disturbance

import time

class ControlLoop():
    """A basic control loop used for running a benchmarking controller.
    """

    def __init__(
        self,
        user_token: str,
        experiment_time: float,
        experiment_type: str,
        cell_id: int | None = None,
        initial_position: list[float] | None = None,
        disturbances: list[Disturbance] | str | None = None,
        dt: float = 0.002,
        record: bool = False,
        preparation_time: float = 5.0,
    ):
        self.user_token: str = user_token
        self.cell_id: int | None = cell_id
        self.experiment_time: float = experiment_time
        self.experiment_type: str = experiment_type
        self.initial_position: list[float] | None = initial_position
        self.disturbances: list[Disturbance] | str | None = disturbances
        self.dt: float = dt
        self.record: bool = record
        self.preparation_time: float = preparation_time

        self.n: int = int(experiment_time / dt)
        self.i: int = 0
        self.loop_start_time: float = 0.0
        self.measured_time: float = 0.0
        self.session_token: str | None = None
        self.client: Client = Client()

    def finished(self) -> bool:
        """Check if the control loop has finished running"""
        return self.measured_time >= self.experiment_time or self.i >= self.n

    def iterations(self) -> int:
        """Get the number of iterations that the control loop has been running
        for
        """
        return self.i

    def time(self) -> float:
        """Get the time since the control loop started, until the current
        control step
        """
        return self.measured_time

    def start(self):
        """Start the control loop by initializing a connection to the cloud
        pendulum server and starting the experiment.
        """
        session_token, livestream_url = self.client.start_experiment(
            user_token = self.user_token,
            experiment_type = self.experiment_type,
            experiment_time = self.experiment_time,
            preparation_time = self.preparation_time,
            initial_position = self.initial_position,
            disturbances = self.disturbances,
            record = self.record,
            cell_id = self.cell_id
        )
        self.session_token = session_token
        self.i = 0
        self.loop_start_time = time.time()
        self.measured_time = 0

    def get_state(self) -> list[MotorState]:
        """Get the current state of the system in the form of a list of
        position/velocity tuples for each actuator in the cell.
        """
        position = self.client.get_position(self.session_token)
        velocity = self.client.get_velocity(self.session_token)
        
        if not isinstance(position, list):
            position = [position]
        if not isinstance(velocity, list):
            velocity = [velocity]

        state = []
        for p, v in zip(position, velocity):
            state.append(
                MotorState(position = p, velocity = v)
            )
        return state

    def get_torque(self) -> list[float]:
        """Get the current torque applied to all the actuators in the cell."""
        torque = self.client.get_torque(self.session_token)
        if not isinstance(torque, list):
            torque = [torque]
        return torque

    def step(self, torques: list[float]):
        """Advance one step in the control loop. Block until next control loop
        iteration is ready.

        :param torques: The control output torques from the controller.
        """
        self.client.set_torque(torques, self.session_token)

        while time.time() - self.loop_start_time < self.dt:
            pass

        current_time = time.time()
        loop_dt = current_time - self.loop_start_time
        self.loop_start_time = current_time
        self.measured_time += loop_dt

        self.i += 1

    def stop(self) -> tuple[str, str]:
        """Stop the control loop. Should be called after `finished` has returned `True`.

        :return: The video url for the experiment if recording was enabled, the
            logs for the experiment.
        """
        logs = self.client.get_logs(self.session_token)
        video_url = self.client.stop_experiment(self.session_token)
        return video_url, logs
