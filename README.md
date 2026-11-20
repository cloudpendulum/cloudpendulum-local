# Cloud Pendulum Local

Driver for local versions of the CloudPendulum single and double pendulums.

## Installation

```
# Create venv and activate it
python3 -m venv CPvenv
source CPvenv/bin/activate
# Install the package (pulls in numpy and pyCandleMAB)
pip install .
```
or for development
```
pip install -e .
```

## Configuration

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

Note: 
1. Your user should be in dialout group
2. Library auto detects single or double pendulum and safety configuration. Can be changed on your own risk in `robot_single.json` resp. `robot.json` or by passing your own parameter dictionary (same structure as `dict(json.load("robot.json"))`).