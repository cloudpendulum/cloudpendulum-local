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

The robot config is read from `$CLOUD_PENDULUM_ROBOT_CONFIG`, else `./robot.json`. A template ships with the package; print its path with `cloud-pendulum-robot-template` and copy it as a starting point.

Check everything worked
```
lsusb | grep MAB
cloud-pendulum-test SimplePendulum # resp. DoublePendulum
```
If your drive is still not recognized, it might be connected to the baud rate the motor driver is configured to. This can be changed permamently using MAB Robotics tools (see their website), or you simply connect with the expected rate:
```
cloud-pendulum-test SimplePendulum --baud-rate 1M # or 2M,5M,8M
```


Note: Your user should be in dialout group
