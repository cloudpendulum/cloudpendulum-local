from .client import Client
from .robot_config import load_config_file
import argparse
import math
import time


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "experiment_type",
        nargs="?",
        default="auto",
        choices=["auto", "SimplePendulum", "SinglePendulum", "DoublePendulum"],
        help="Which system to run the test against. Default 'auto' detects it "
             "from the number of connected motors (1 = single, 2 = double). "
             "SimplePendulum and SinglePendulum refer to the same "
             "(single-actuator) system.",
    )
    parser.add_argument(
        "--baud-rate",
        default=None,
        choices=["1M", "2M", "5M", "8M"],
        help="CAN baud rate to use, overriding robot.json's \"can-baud-rate\". "
             "Defaults to whatever robot.json specifies.",
    )
    args = parser.parse_args(argv)
    experiment_type = args.experiment_type

    baud = int(args.baud_rate.rstrip("M")) if args.baud_rate is not None else None

    if experiment_type == "auto":
        # Client detects single/double pendulum from the connected motors and
        # prints which one it chose.
        client = Client(can_baud_rate=baud, verbose=True)
    else:
        # Explicit choice: use the matching config file (see robot_config)
        # and require matching hardware.
        config_name = "robot.json" if experiment_type == "DoublePendulum" \
            else "robot_single.json"
        client = Client(load_config_file(config_name), can_baud_rate=baud,
                        verbose=True)

    joints = client.robot_config["robot-joints"]
    is_double_pendulum = len(joints) > 1
    # Joints without a pinned "id" were auto-detected on the bus.
    # (Client fills in the ids, so look at the config file instead.)
    auto_detected_joints = [
        j.get("id") is None
        for j in load_config_file("robot.json")["robot-joints"]
    ][:len(joints)]
    client.start_experiment()

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
            "current work folder (or point $CLOUD_PENDULUM_CONFIG_DIR at a folder "
            "containing one) specifying the correct ids. Use the template "
            "shipped with the package (folder: `cloud-pendulum-robot-template`) "
            "as a base, adding the \"id\": <number> attribute to each joint."
        )

    Tf = 5.0
    kp = 0.20
    kd = 0.01
    if is_double_pendulum:
        client.set_impedance_controller_params([kp, kp], [kd, kd])
    else:
        client.set_impedance_controller_params([kp], [kd])

    dt = 0.005

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


if __name__ == "__main__":
    main()
