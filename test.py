from cloud_pendulum_local import Client
import argparse
import json
import math
import time

parser = argparse.ArgumentParser()
parser.add_argument(
    "experiment_type",
    nargs="?",
    default="SimplePendulum",
    choices=["SimplePendulum", "SinglePendulum", "DoublePendulum"],
    help="Which system to run the test against. SimplePendulum and "
         "SinglePendulum refer to the same (single-actuator) system.",
)
parser.add_argument(
    "--baud-rate",
    default=None,
    choices=["1M", "2M", "5M", "8M"],
    help="CAN baud rate to use, overriding robot.json's \"can-baud-rate\". "
         "Defaults to whatever robot.json specifies.",
)
args = parser.parse_args()
experiment_type = args.experiment_type
is_double_pendulum = experiment_type == "DoublePendulum"

if not is_double_pendulum:
    print("Defaulting to SinglePendulum. If you have DoublePendulum hardware instead, this can lead to confusing error messages.")

# Read robot.json ourselves (before Client() mutates its own copy) so we can
# tell which joints had no "id" pinned, i.e. were auto-detected on the bus.
with open("robot.json", "r") as f:
    _robot_config = json.load(f)
if not is_double_pendulum:
    # single-actuator hardware: only the first joint is connected
    _robot_config["robot-joints"] = _robot_config["robot-joints"][:1]
auto_detected_joints = [
    joint.get("id") is None for joint in _robot_config["robot-joints"]
]

if args.baud_rate is not None:
    _robot_config["can-baud-rate"] = int(args.baud_rate.rstrip("M"))

client = Client(_robot_config)
client.start_experiment()

joints = client.robot_config["robot-joints"]

if is_double_pendulum and len(joints) > 1 and any(auto_detected_joints):
    labels = ["inner", "outer"]
    directions = ["clockwise", "counterclockwise"]
    print(
        "Debug: auto-detected actuators. Actuators should move in opposite "
        "directions (positive = counterclockwise, negative = clockwise) "
        "resulting in the inner leg moving, the outer pointing down:"
    )
    for joint, label, direction in zip(joints, labels, directions):
        print(f"  actuator {joint['id']} is {label} pendulum and should move {direction}.")
    print(
        "If they move the same way instead, create a robot.json in your "
        "current work folder specifying the correct ids. You can use "
        "robot.json (in this directory) as a template, adding the "
        "\"id\": <number> attribute to each joint."
    )

Tf = 5.0
kp = 0.20
kd = 0.01
if is_double_pendulum:
    client.set_impedance_controller_params([kp, kp], [kd, kd])
else:
    client.set_impedance_controller_params([kp], [kd])

dt = 0.005
n=int(Tf/dt)

t = 0.0

i = 0
meas_dt = 0.0
meas_time = 0.0

max_freq = 0.0
min_freq = math.inf
avg_freq = 0.0

while meas_time < Tf:
    start_loop = time.time()
    meas_time += meas_dt

    if is_double_pendulum:
        # inner actuator moves clockwise (negative), outer counterclockwise (positive)
        client.set_position([-meas_time * 4.0, meas_time * 4.0])
    else:
        client.set_position(meas_time * 1.0)

    client.get_position()
    client.get_velocity()
    client.get_torque()

    i += 1
    exec_time = time.time() - start_loop
    min_freq = min(min_freq, 1.0 / exec_time)
    max_freq = max(max_freq, 1.0 / exec_time)
    avg_freq = avg_freq + 1.0 / exec_time
    while time.time() - start_loop < dt:
        pass
    meas_dt = time.time() - start_loop

avg_freq = avg_freq / float(i)

print(
    "Finished",
    "- avg frequency:", avg_freq,
    " - min frequency:", min_freq,
    " - max frequency:", max_freq
)

client.stop_experiment()
