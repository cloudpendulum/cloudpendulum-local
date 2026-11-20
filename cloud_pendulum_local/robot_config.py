"""Locating and loading robot configuration files (robot.json, robot_single.json)."""
import json
import os
from importlib import resources

ENV_VAR = "CLOUD_PENDULUM_CONFIG_DIR"


def package_config_path(name: str) -> str:
    """Path of the config template `name` shipped with the package."""
    return str(resources.files("cloud_pendulum_local").joinpath(name))


def find_config_file(name: str) -> str:
    """Locate config file `name`: $CLOUD_PENDULUM_CONFIG_DIR, then the current
    directory, then the template shipped with the package."""
    env_dir = os.environ.get(ENV_VAR)
    if env_dir and os.path.exists(os.path.join(env_dir, name)):
        return os.path.join(env_dir, name)
    if os.path.exists(name):
        return name
    return package_config_path(name)


def load_config_file(name: str) -> dict:
    with open(find_config_file(name), "r") as f:
        return json.load(f)


def print_template_path() -> None:
    """Console entry point: print the directory holding the shipped templates."""
    print(os.path.dirname(package_config_path("robot.json")))
