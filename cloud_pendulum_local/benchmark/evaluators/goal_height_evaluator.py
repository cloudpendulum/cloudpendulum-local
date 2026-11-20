from ..evaluator import Evaluator
from ..controller import MotorState

import math

class GoalHeightEvaluator(Evaluator):
    """An evaluator that scores a controller based on the amount of time a
    pendulum manages to stay above a certain height. Works for pendulums with
    any number of joints connected in sequence.
    """

    def __init__(
        self,
        goal_height_scale: float = 0.90,
        goal_time_threshold: float = 1.0,
        verbose: bool = True
    ):
        """Initialize the evaluator.

        :param goal_height_scale: The fraction of the maximum height at which
            the pendulum is considered to be in the goal position.
        :param goal_time_threshold: The minimum time the pendulum has to be
            above the goal height before it updates the score. For example: if
            goal_time_threshold = 0.9 and the pendulum stays above the goal
            height for 1 second before falling down the score will be increased
            by 1. If the goal_time_threshold = 1.1 then the score would not be
            updated.
        :param verbose: Print to stdout whether or not the pendulum is in the
            goal position.
        """
        self.goal_height_scale: float = goal_height_scale
        self.swingup_time: float | None = None
        self.goal_time: float | None = None
        self.prev_time: float | None = None
        self.score: float = 0.0
        self.goal_time_threshold: float = goal_time_threshold
        self.verbose = verbose

    def evaluate(
        self,
        state: list[MotorState],
        controller_output: list[float],
        time: float,
    ):
        angle = 0.0
        height = 0.0
        for s in state:
            angle += s.position
            height += -math.cos(angle)
        goal_height = float(len(state)) * self.goal_height_scale

        if height >= goal_height:
            if self.goal_time is None:
                if self.verbose:
                    print("IN GOAL")
                self.goal_time = time
        else:
            if self.goal_time is not None:
                if self.verbose:
                    print("NOT IN GOAL")
            self.goal_time = None
            
        if self.goal_time is not None and self.prev_time is not None:
            time_in_goal = time - self.goal_time
            prev_time_in_goal = self.prev_time - self.goal_time
            if time_in_goal >= self.goal_time_threshold:
                if prev_time_in_goal < self.goal_time_threshold and prev_time_in_goal >= 0.0:
                    self.score += prev_time_in_goal
                self.score += time - self.prev_time
        self.prev_time = time

    def get_score(self) -> float:
        return self.score

