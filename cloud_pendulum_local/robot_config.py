"""Locating and loading the robot configuration (robot.json)."""
import json
import os
from importlib import resources

ENV_VAR = "CLOUD_PENDULUM_ROBOT_CONFIG"
FILE_NAME = "robot.json"


def template_path() -> str:
    """Path of the robot.json template shipped with the package."""
    return str(resources.files("cloud_pendulum_local").joinpath(FILE_NAME))


def find_robot_config() -> str:
    """Return the robot config path: $CLOUD_PENDULUM_ROBOT_CONFIG, else ./robot.json."""
    env = os.environ.get(ENV_VAR)
    if env:
        return env
    if os.path.exists(FILE_NAME):
        return FILE_NAME
    raise FileNotFoundError(
        f"No {FILE_NAME} found in the current directory and ${ENV_VAR} is not set. "
        f"Copy the template from {template_path()} and adapt it "
        f"(run `cloud-pendulum-robot-template` to print its path)."
    )


def load_robot_config(path: str | None = None) -> dict:
    with open(path or find_robot_config(), "r") as f:
        return json.load(f)


def print_template_path() -> None:
    print(template_path())
