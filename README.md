# Cloud Pendulum Local

Driver for local versions of the CloudPendulum single and double pendulums.

## Installation

```
# Create venv and activate it
python3 -m venv CPvenv
source CPvenv/bin/activate
# Install the package (pulls in numpy and pyCandleMAB)
pip install .   # or `pip install -e .` for development
```

The `import pyCandle` workaround for the C++ library linking bug in the MAB motor driver is done automatically when the package is imported.

## Configuration

Without an explicit config, `Client()` detects the system from the number of motors on the CAN bus: 1 motor uses `robot_single.json` (single pendulum), 2 motors use `robot.json` (double pendulum). These files are searched in `$CLOUD_PENDULUM_CONFIG_DIR`, then the current directory, then the templates shipped with the package. Print the template folder with `cloud-pendulum-robot-template` and copy a file from there as a starting point.

Check everything worked
```
lsusb | grep MAB
cloud-pendulum-test            # auto-detects single/double pendulum
cloud-pendulum-test SimplePendulum # or DoublePendulum to force a type
```
If your drive is still not recognized, it might be connected to the baud rate the motor driver is configured to. This can be changed permamently using MAB Robotics tools (see their website), or you simply connect with the expected rate:
```
cloud-pendulum-test SimplePendulum --baud-rate 1M # or 2M,5M,8M
```


Note: Your user should be in dialout group
