from __future__ import annotations
from dataclasses import dataclass, asdict, field
from typing import Optional, Any
import json
import warnings
import numpy as np

warnings.warn(
    "The CloudPendulumLocal library knows about all the disturbances, "
    "but the client will ignore these arguments"
)

# class DisturbanceType(Enum):
#     TIMED_TORQUE = 1,
#     STATE_TORQUE = 2,
#     TIMED_POSITION = 3,
#     STATE_POSITION = 4,

@dataclass 
class Disturbance:
    """General disturbance"""
    pass

    # def serialize(self): -> list[Any]
    #     return [int(self.disturbance_type())] + self._serialize()

    # @staticmethod
    # def deserialize(data: list[Any]): -> (Disturbance, int)
    #     assert len(data) >= 1
    #     if data[0] == DisturbanceType.TIMED_TORQUE:
    #         return TimedTorqueDisturbance._deserialize(data[1:])
    #     elif data[0] == DisturbanceType.STATE_TORQUE:
    #         return StateTorqueDisturbance._deserialize(data[1:])
    #     elif data[0] == DisturbanceType.TIMED_POSITION:
    #         return TimedPositionDisturbance._deserialize(data[1:])
    #     elif data[0] == DisturbanceType.TIMED_TORQUE:
    #         return TimedTorqueDisturbance._deserialize(data[1:])
    #     else:
    #         raise NotImplementedError()

    # def _serialize(self): -> list[Any]
    #     raise NotImplementedError()
    # @staticmethod
    # def _deserialize(data: list[Any]): -> (Disturbance, int)
    #     raise NotImplementedError()

    # def disturbance_type(self) -> DisturbanceType:
    #     raise NotImplementedError()

@dataclass 
class TimedTorqueDisturbance(Disturbance):
    """Apply extra torque additively at `start_time` for `duration` seconds.
    One torque value per actuator.

    :param start_time: Start time in seconds after experiment start
    :param duration: Duration in seconds for how long the torque is applied
    :param torque: List of torque values, one per actuator
    """
    start_time: float
    duration: float
    torque: list[float]
    type: str = field(default="timed_torque", init=False) # Tag used for JSON serialization

    # def _serialize(self): -> list[Any]
    #     num_torque = len(self.torque)
    #     return [duration, num_torque] + torque

    # @staticmethod
    # def _deserialize(data: list[Any]): -> (TimedTorqueDisturbance, int)
    #     assert(len(data) >= 2 and len(data) >= 2 + data[1])
    #     torque_start = 2
    #     torque_len = data[1]
    #     torque_end = torque_start + torque_len
    #     return (
    #         TimedTorqueDisturbance (
    #             duration = data[0],
    #             torque = data[torque_start:torque_end]
    #         ),
    #         torque_end
    #     )

@dataclass
class StateDependentPositionDisturbance(Disturbance):
    """Trigger a position jump after staying near `goal_position` for
    `wait_at_target_seconds`.

    :param wait_at_target_seconds: How long the system stays at goal_position
        before a disturbance is triggered
    :param goal_position: The controller target position that the system is 
        supposed to reach. If `goal_position` is None, `initial_position` from
        start_experiment is used as the reference and must be set.
    :param disturbance_position: The position that the disturbance will drive
        the system to. If `disturbance_position` is None, the position will
        be randomized.
    :param target_pos_threshold: Admissible distance error to `goal_position` 
        to be detected as "at goal"
    :param target_vel_threshold: Admissible velocity error to be detected as 
        "at goal"
    """
    wait_at_target_seconds: float = 3.0
    goal_position: Optional[list[float]] = None # None = use initial_position as reference
    disturbance_position: Optional[list[float]] = None  # None = randomize within safety limits
    target_pos_threshold: float = 0.1 # position error (rad)
    target_vel_threshold: float = 1.0 # velocity error (rad/s)
    type: str = field(default="state_dependent_position", init=False) # Tag used for JSON serialization

    # def _serialize(self): -> list[Any]
    #     goal_pos = self.goal_position or []
    #     disturbance_pos = self.disturbance_position or []
    #     return [wait_at_target_seconds] +\
    #         [len(goal_pos)] + goal_pos +\
    #         [len(disturbance_position)] + disturbance_position +\
    #         [target_pos_threshold, target_vel_threshold]

    # @staticmethod
    # def _deserialize(data: list[Any]): -> (TimedTorqueDisturbance, int)
    #     start_goal_pos = 2
    #     assert len(data) >= start_goal_pos
    #     num_goal_pos = data[start_goal_pos - 1]
    #     end_goal_pos = start_goal_pos + num_goal_pos

    #     start_disturbance_pos = end_goal_pos + 1
    #     assert len(data) >= start_disturbance_pos
    #     num_disturbance_pos = data[start_disturbance_pos - 1]
    #     end_disturbance_pos = start_disturbance_pos + num_disturbance_pos

    #     assert len(data) >= end_disturbance_pos + 2

    #     return (
    #         StateDependentPositionDisturbance (
    #             wait_at_target_seconds = data[0],
    #             goal_position = data[start_goal_pos:end_goal_pos],
    #             disturbance_position = data[start_disturbance_pos:end_disturbance_pos],
    #             target_pos_threshold: data[end_disturbance_pos],
    #             target_vel_threshold: data[end_disturbance_pos + 1]
    #         ),
    #         end_disturbance_pos + 2
    #     )

@dataclass
class StateDependentTorqueDisturbance(Disturbance):
    """Trigger a torque for `duration` seconds after staying near
    `goal_position` for `wait_at_target_seconds`.

    :param wait_at_target_seconds: How long the system stays at goal_position
        before a disturbance is triggered
    :param goal_position: The controller target position that the system is 
        supposed to reach. If `goal_position` is None, `initial_position` from
        start_experiment is used as the reference and must be set.
    :param torque: List of torque values, one per actuator
    :param target_pos_threshold: Admissible distance error to `goal_position` 
        to be detected as "at goal"
    :param target_vel_threshold: Admissible velocity error to be detected as 
        "at goal"
    """
    duration: float
    torque: list[float]
    wait_at_target_seconds: float = 3.0
    goal_position: Optional[list[float]] = None # None = use initial_position as reference
    target_pos_threshold: float = 0.1 # position error (rad)
    target_vel_threshold: float = 1.0 # velocity error (rad/s)
    type: str = field(default="state_dependent_torque", init=False) # Tag used for JSON serialization


@dataclass
class TimedPositionDisturbance(Disturbance):
    """Jump to target position at `trigger_time` seconds after experiment start.

    :param trigger_time: At which time after experiment start the disturbance 
        should be triggered.
    :param disturbance_position: The position that the disturbance will drive
        the system to. If `disturbance_position` is None, the position will
        be randomized.
    """
    trigger_time: float
    disturbance_position: Optional[list[float]] = None  # None = randomize within safety limits
    type: str = field(default="timed_position", init=False) # Tag used for JSON serialization

    # def _serialize(self): -> list[Any]
    #     raise NotImplementedError()
    # @staticmethod
    # def _deserialize(data: list[Any]): -> (TimedPositionDisturbance, int)
    #     raise NotImplementedError()

@dataclass
class DisturbanceSchema:
    """Class used internally to carry named disturbances such as 'double_pendulum_default'
    
    :param name: name of the disturbance schema"""
    name: str
    type: str = field(default="disturbance_schema", init=False) # Tag used for JSON serialization


def serialize_disturbances(disturbances: Disturbance | str | list[Disturbance | str]) -> str:
    if not isinstance(disturbances, list):
        disturbances = [disturbances]
    dists: list[Disturbance | DisturbanceSchema] = [
        DisturbanceSchema(name=d) if isinstance(d, str) else d for d in disturbances
    ]
    for i in range(len(dists)):
        if not (isinstance(dists[i], Disturbance) or isinstance(dists[i], DisturbanceSchema)):
            raise RuntimeError("Invalid disturbance")

    # Convert potential numpy arrays to list
    for dist in dists:
        for field in dist.__dataclass_fields__:
            value = getattr(dist, field)
            if isinstance(value, np.ndarray):
                setattr(dist, field, list(value.squeeze()))

    return json.dumps([asdict(d) for d in dists])

def deserialize_disturbance(json_data: dict[Any,Any]) -> Disturbance:
    t = json_data["type"]
    kwargs = {k: v for k, v in json_data.items() if k not in ("type", "end_time")}
    if t == "state_dependent_position":
        return StateDependentPositionDisturbance(**kwargs)
    if t == "state_dependent_torque":
        return StateDependentTorqueDisturbance(**kwargs)
    elif t == "timed_position":
        return TimedPositionDisturbance(**kwargs)
    elif t == "timed_torque":
        return TimedTorqueDisturbance(**kwargs)
    else:
        raise RuntimeError("Invalid disturbance type")

# def serialize_disturbances(disturbances: list[str | Disturbance]): -> list[Any]
#     result = []
#     for disturbance in disturbances:
#         if isinstance(disturbance, Disturbance):
#             result += disturbance.serialize()
#         elif isinstance(disturbance, str):
#             result += disturbance
#         else:
#             raise ValueError("Invalid disturbance")
#     return result

