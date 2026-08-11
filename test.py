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
args = parser.parse_args()
experiment_type = args.experiment_type
is_double_pendulum = experiment_type == "DoublePendulum"

# Read robot.json ourselves (before Client() mutates its own copy) so we can
# tell which joints had no "id" pinned, i.e. were auto-detected on the bus.
with open("robot.json", "r") as f:
    _robot_config = json.load(f)
auto_detected_joints = [
    joint.get("id") is None for joint in _robot_config["robot-joints"]
]

client = Client()
client.start_experiment()

joints = client.robot_config["robot-joints"]

if is_double_pendulum and len(joints) > 1 and any(auto_detected_joints):
    labels = ["inner", "outer"]
    directions = ["clockwise", "counterclockwise"]
    print(
        "Debug: auto-detected actuators. Actuators should move in opposite "
        "directions (positive = counterclockwise, negative = clockwise):"
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

dt = 0.002
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
        client.set_position(meas_time * 4.0)

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
