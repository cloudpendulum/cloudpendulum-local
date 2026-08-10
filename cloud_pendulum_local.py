import pyCandle
import time
import json
import math
import numpy
from dataclasses import dataclass

@dataclass
class UserInfo:
    user_name: str
    max_experiment_length: int
    number_of_attempts: int
    max_bucket_size: int
    bucket_size: int
    refil_interval: int
    valid_from: int
    valid_until: int
    experiment_types: [str]

@dataclass
class Actuator:
    """Information about an actuator, specific to a certain experiment type.
    """
    name: str
    pos_limit: float
    vel_limit: float
    torque_limit: float

candle_instance = None
motor_ids_instance = None

def CubicTimeScaling(Tf, t):
    """Computes s(t) for a cubic time scaling
    Source: Modern Robotics Toolbox (https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py#L1455C1-L1469C61)
    :param Tf: Total time of the motion in seconds from rest to rest
    :param t: The current time t satisfying 0 < t < Tf
    :return: The path parameter s(t) corresponding to a third-order
             polynomial motion that begins and ends at zero velocity

    Example Input:
        Tf = 2
        t = 0.6
    Output:
        0.216
    """
    return 3 * (1.0 * t / Tf) ** 2 - 2 * (1.0 * t / Tf) ** 3

def JointTrajectory(thetastart, thetaend, Tf, N):
    """Computes a straight-line trajectory in joint space
    Source: Modern Robotics Toolbox (modified) 
    :param thetastart: The initial joint variables
    :param thetaend: The final joint variables
    :param Tf: Total time of the motion in seconds from rest to rest
    :param N: The number of points N > 1 (Start and stop) in the discrete
              representation of the trajectory
    :return: A trajectory as an N x n matrix, where each row is an n-vector
             of joint variables at an instant in time. The first row is
             thetastart and the Nth row is thetaend . The elapsed time
             between each row is Tf / (N - 1)

    Example Input:
        thetastart = numpy.array([1, 0, 0, 1, 1, 0.2, 0,1])
        thetaend = numpy.array([1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1])
        Tf = 4
        N = 6
        method = 3
    Output:
        numpy.array([[     1,     0,      0,      1,     1,    0.2,      0, 1]
                  [1.0208, 0.052, 0.0624, 1.0104, 1.104, 0.3872, 0.0936, 1]
                  [1.0704, 0.176, 0.2112, 1.0352, 1.352, 0.8336, 0.3168, 1]
                  [1.1296, 0.324, 0.3888, 1.0648, 1.648, 1.3664, 0.5832, 1]
                  [1.1792, 0.448, 0.5376, 1.0896, 1.896, 1.8128, 0.8064, 1]
                  [   1.2,   0.5,    0.6,    1.1,     2,      2,    0.9, 1]])
    """
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = numpy.zeros((len(thetastart), N))
    for i in range(N):
        s = CubicTimeScaling(Tf, timegap * i)
        traj[:, i] = s * numpy.array(thetaend) + (1 - s) * numpy.array(thetastart)
    traj = numpy.array(traj).T
    return traj

class Client:
    """Client interface for running a cloud pendulum experiment on hardware
    locally. The experiment is configured through a `robot.json` file which
    should have the following format:

    ```
    {
        "can-baud-rate": <1, 2, 5 or 8>,
        "robot-joints": [
            {
                "id": <MOTOR_ID, or omit/null to auto-detect>,
                "pos_limit": <POSITION LIMIT IN RADIANS>,
                "vel_limit": <VELOCITY LIMIT IN RAD/s>,
                "torque_limit": <TORQUE LIMIT IN Nm>,
                "name": <SOME IDENTIFIER FOR DEBUGGING>
            },
            ...
        ]
    }
    ```

    `id` is optional. If a joint omits it (or sets it to `null`), that joint
    is auto-assigned to a motor found on the CAN bus, the same way
    `pendulum_plant.py`'s `activate_hardware()` does it -- ping the bus and
    take whatever's connected. Joints that do specify an `id` are matched
    against that exact motor as before (this is the original, backwards
    compatible behavior); auto-assigned joints then claim whichever
    remaining discovered motors aren't already spoken for by an explicit
    `id`, in the order `robot-joints` lists them. For multi-joint setups
    this order isn't guaranteed to match physical wiring -- prefer explicit
    `id`s if joint identity matters, and check the "Auto-assigned motor ..."
    printout at startup either way.

    The local client interface is made to be compatible with the client
    interface for the server. Therfore, most methods take parameter
    `session_token`, which is unused in this implementation. All method
    parameters that have a default value (like the `session_token`) are unused
    and can be ignored.
    """

    def __init__(self) -> None:
        global candle_instance
        global motor_ids_instance

        self.robot_config = {}
        with open(f"robot.json", "r") as f:
            self.robot_config = json.load(f)

        can_baud_rate_config = self.robot_config["can-baud-rate"]
        if candle_instance is None:
            can_baud_rate = None
            if can_baud_rate_config == 1:
                can_baud_rate = pyCandle.CAN_BAUD_1M
            elif can_baud_rate_config == 2:
                can_baud_rate = pyCandle.CAN_BAUD_2M
            elif can_baud_rate_config == 5:
                can_baud_rate = pyCandle.CAN_BAUD_5M
            elif can_baud_rate_config == 8:
                can_baud_rate = pyCandle.CAN_BAUD_8M
            else:
                raise RuntimeError(f"Invalid baudrate: {can_baud_rate_config}, should be 1, 2, 5 or 8")

            manual_ids = {
                joint["id"] for joint in self._robot_joints()
                if joint.get("id") is not None
            }
            auto_joint_count = sum(
                1 for joint in self._robot_joints() if joint.get("id") is None
            )

            while True:
                try:
                    candle_instance = \
                        pyCandle.Candle(can_baud_rate, True, pyCandle.USB)
                    motor_ids_instance = candle_instance.ping(can_baud_rate)

                    # manually-pinned joints must all be present, and there
                    # must be enough leftover motors to cover the
                    # auto-detected joints
                    available_for_auto = [
                        id for id in motor_ids_instance if id not in manual_ids
                    ]
                    valid = manual_ids.issubset(set(motor_ids_instance)) \
                        and len(available_for_auto) >= auto_joint_count

                    if valid:
                        for id in motor_ids_instance:
                            candle_instance.addMd80(id)
                        break
                except:
                    raise RuntimeError("No valid candle")

        self.candle = candle_instance
        self.all_motor_ids = motor_ids_instance

        claimed_ids = {
            joint["id"] for joint in self._robot_joints()
            if joint.get("id") is not None
        }
        available_for_auto = [
            id for id in self.all_motor_ids if id not in claimed_ids
        ]
        next_auto = 0
        for joint in self._robot_joints():
            if joint.get("id") is not None:
                if joint["id"] not in self.all_motor_ids:
                    raise RuntimeError(f"Motor with id {joint['id']} is not connected")
                continue

            if next_auto >= len(available_for_auto):
                raise RuntimeError(
                    f"Could not auto-assign a motor to joint "
                    f"'{joint.get('name', '?')}': not enough connected motors "
                    f"({len(self.all_motor_ids)} found, {len(claimed_ids)} "
                    f"explicitly assigned in robot.json)"
                )
            joint["id"] = available_for_auto[next_auto]
            next_auto += 1
            print(f"Auto-assigned motor {joint['id']} to joint '{joint.get('name', '?')}'")

    def go_to_state(self, x0: list[float]):
        actuator_ids = []
        for joint in self._robot_joints():
            actuator_ids.append(joint["id"])
        thetastart = []
        for actuator_id in actuator_ids:
            thetastart.append(self._get_motor(actuator_id).getPosition())
        thetaend = x0
        dt0 = 0.01
        tf0 = 2.0
        N0 = int(tf0 / dt0)
        traj = JointTrajectory(thetastart, thetaend, tf0, N0)
        for actuator_id in actuator_ids:
            # self.motor_driver.set_impedance_controller_params_unchecked(actuator_id, 0.25, 0.0015)
            motor = self._get_motor(actuator_id)
            motor.setImpedanceControllerParams(0.25, 0.0015)
        i = 0
        current_time = time.time()
        target_time = current_time
        while i < N0:
            for j, actuator_id in enumerate(actuator_ids):
                # self.motor_driver.set_target_position_unchecked(actuator_id, traj[i][j])
                self._get_motor(actuator_id).setTargetPosition(traj[i][j])
            i += 1
            target_time += dt0
            current_time = time.time()
            time.sleep(target_time - current_time)

    def start_experiment(
        self,
        user_token: str = "",
        experiment_type: str = "",
        experiment_time: float = 0.0,
        preparation_time: float = 0.0,
        record: bool = False,
        initial_state: list[float] | None = None,
    ) -> tuple[str, str]:
        """Start the experiment by starting the candle loop

        :return: Tuple of empty strings, for consistency with the remote
            client.
        """
                
        for md in self.candle.md80s:
            self.candle.controlMd80SetEncoderZero(md)
            self.candle.controlMd80Mode(md, pyCandle.IMPEDANCE)
            md.setImpedanceControllerParams(0.0, 0.0);
            self.candle.controlMd80Enable(md, True)
        self.candle.begin()
        time.sleep(1.0)
        if initial_state is not None:
            self.go_to_state(initial_state)
        return "", ""

    def stop_experiment(self, session_token: str = "") -> str:
        """Stop the experiment by restoring the motors to their resting
        position and ending the candle loop

        :return: Empty string, for consistency with the remote client.
        """
        self._reset_motors()
        self.candle = None
        return ""

    def get_user_info(self, user_token: str = "") -> UserInfo:
        """Unused method. Included for consistency with the remote client
        interface.

        :return: Default `UserInfo`, for consistency with the remote client.
        """
        return UserInfo(
            user_name = "",
            max_experiment_length = 0.0,
            number_of_attempts = 0,
            max_bucket_size = 0,
            bucket_size = 0,
            refil_interval = 0,
            valid_from = 0,
            valid_until = 0,
            experiment_types = []
        )

    def get_actuator_info(self, session_token: str = "") -> list[Actuator]:
        """Get information about the actuator for the experiment specified in
        robots.json

        :return: A list of `Actuator`s.
        """
        def create_actuator(joint: Any) -> Actuator:
            return Actuator(
                name = joint["name"],
                pos_limit = joint["pos_limit"],
                vel_limit = joint["vel_limit"],
                torque_limit = joint["torque_limit"]
            )

        return list(map(lambda j: create_actuator, self._robot_joints()))

    def get_position(self, session_token: str = "") -> list[float] | float:
        """Get the positions of the actuators.

        :return: The position of the actuators. The positions are returned as a
            list if the experiment has more than one actuator. Otherwise, a
            single number is returned.
        """
        res: list[float] = []
        for joint in self._robot_joints():
            motor_id = joint["id"]
            pos = self._get_motor(motor_id).getPosition()
            pos_limit = joint["pos_limit"]
            if not -pos_limit < pos < pos_limit:
                self._raise_exception(f"PositionLimitViolation;value={pos}")
            res = res + [pos]
        return self._prepare_output_data(res)

    def get_velocity(self, session_token: str = "") -> list[float] | float:
        """Get the velocities of the actuators.

        :return: The velocities of the actuators. The velocities are returned
            as a list if the experiment has more than one actuator. Otherwise,
            a single number is returned.
        """
        res: list[float] = []
        for joint in self._robot_joints():
            motor_id = joint["id"]
            vel = self._get_motor(motor_id).getVelocity()
            vel_limit = joint["vel_limit"]
            if not -vel_limit < vel < vel_limit:
                self._raise_exception(f"VelocityLimitViolation;value={vel}")
            res = res + [vel]
        return self._prepare_output_data(res)

    def get_torque(self, session_token: str = "") -> list[float] | float:
        """Get the torques of the actuators.

        :return: The torques of the actuators. The torques are returned
            as a list if the experiment has more than one actuator. Otherwise,
            a single number is returned.
        """
        res: list[float] = []
        for joint in self._robot_joints():
            motor_id = joint["id"]
            torque = self._get_motor(motor_id).getTorque()
            torque_limit = joint["torque_limit"]
            if not -torque_limit < torque < torque_limit:
                self._raise_exception(f"TorqueLimitViolation;value={torque}")
            res = res + [torque]
        return self._prepare_output_data(res)

    def set_position(self, positions: list[float | None] | float, session_token: str = ""):
        """Set the target positions for the actuators.

        :param positions: The target positions for the actuators. If None is
            supplied for an actuator, it is ignored. A single number can be
            supplied for experiments that only use a single motor.
        """
        positions = self._prepare_actuator_data(positions)
        for position, joint in zip(positions, self._robot_joints()):
            if position is None:
                continue
            pos_limit = joint["pos_limit"]
            clamped_position = min(pos_limit, max(-pos_limit, position))
            self._get_motor(joint["id"]).setTargetPosition(clamped_position)
            if pos_limit < abs(position):
               print(f"Position clamped from {position} to {clamped_position}")

    def set_velocity(self, velocities: list[float | None] | float, session_token: str = ""):
        """Set the target velocities for the actuators.

        :param velocities: The target velocities for the actuators. If None is
            supplied for an actuator, it is ignored. A single number can be
            supplied for experiments that only use a single motor.
        """
        velocities = self._prepare_actuator_data(velocities)
        for velocity, joint in zip(velocities, self._robot_joints()):
            if velocity is None:
                continue
            vel_limit = joint["vel_limit"]
            clamped_velocity = min(vel_limit, max(-vel_limit, velocity))
            self._get_motor(joint["id"]).setTargetVelocity(clamped_velocity)
            if vel_limit < abs(velocity):
                print(f"Velocity clamped from {velocity} to {clamped_velocity}")

    def set_torque(self, torques: list[float | None] | float, session_token: str = ""):
        """Set the target torques for the actuators.

        :param velocities: The target torques for the actuators. If None is
            supplied for an actuator, it is ignored. A single number can be
            supplied for experiments that only use a single motor.
        """
        torques = self._prepare_actuator_data(torques)
        for torque, joint in zip(torques, self._robot_joints()):
            if torque is None:
                continue
            torque_limit = joint["torque_limit"]
            clamped_torque = min(torque_limit, max(-torque_limit, torque))
            self._get_motor(joint["id"]).setTargetTorque(clamped_torque)
            if torque_limit < abs(torque):
                print(f"Torque clamped from {torque} to {clamped_torque}")

    def set_impedance_controller_params(
        self,
        kp: float | list[float | None],
        kd: float | list[float | None],
        session_token: str = ""
    ):
        """Set the impedance controller parameters of the actuators.

        :param kp: Displacement gain for each motor. Applies to all motors if
            a single number is supplied.
        :param kd: Damping coefficient for each motor. Applies to all motors if
            a single number is supplied.
        """
        kp = self._prepare_actuator_data_fill(kp)
        kd = self._prepare_actuator_data_fill(kd)
        for p, d, joint in zip(kp, kd, self._robot_joints()):
            if p is None or d is None:
                continue
            motor = self._get_motor(joint["id"])
            motor.setImpedanceControllerParams(p, d)

    def _robot_joints(self):
        return self.robot_config["robot-joints"]

    def _get_motor(self, id: int) -> pyCandle.Md80:
        for md in self.candle.md80s:
            if md.getId() == id:
                return md
        self._raise_exception(f"No motor with id {id} connected")

    def _reset_motors(self):
        for motor in self.candle.md80s:
            pos = motor.getPosition()
            offset = round(pos / (2.0 * math.pi)) * (2.0 * math.pi)
            motor.setImpedanceControllerParams(0.05, 0.005)
            motor.setTargetTorque(0.0)
            motor.setTargetPosition(offset)
            motor.setTargetVelocity(0.0)
        time.sleep(2.0)
        self.candle.end()

    def _prepare_actuator_data(
        self,
        data: float | list[float | None]
    ) -> list[float | None]:
        if isinstance(data, numpy.ndarray):
            data = data.tolist()
        if not isinstance(data, list):
            data = [data]
        for d in data:
            if d is not None:
                d = float(d)
        if len(data) != len(self._robot_joints()):
            self._raise_exception(f"Wrong number of arguments supplied; Expected {len(self._robot_joints())}, got {len(data)}")
        return data

    def _prepare_actuator_data_fill(
                self,
        data: float | list[float | None]
    ) -> list[float | None]:
        if isinstance(data, numpy.ndarray):
            data = data.tolist()
        if not isinstance(data, list):
            data = [data]
        for d in data:
            if d is not None:
                d = float(d)
        if len(data) == 1:
            data = data * len(self._robot_joints())
        return data

    def _prepare_output_data(self, data: list[float]) -> list[float] | float:
        if len(data) == 1:
            return data[0]
        return data

    def _raise_exception(self, message):
        self._reset_motors()
        raise RuntimeError(message)
