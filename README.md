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
```
This means you can see the motor driver.

Next, check if `dialout` and `uucp` are in your user's groups:
```
groups
# should show among others dialout, uucp
```
If it doesn't, add your user to these groups
```
sudo usermod -aG dialout $USER
sudo usermod -aG uucp $USER
```
and reload the groups (alternatively, log yourself out and in again)
```
su -l $USER
```
Test that you can actuate the motor:
```
cloud-pendulum-test            # auto-detects single/double pendulum
cloud-pendulum-test SimplePendulum # or DoublePendulum to force a type
```
If your drive is still not recognized, it might be connected to the baud rate the motor driver is configured to. This can be changed permanently using MAB Robotics tools (see their website), or you simply connect with the expected rate:
```
cloud-pendulum-test SimplePendulum --baud-rate 1M # or 2M,5M,8M
```

Note: 
1. Your user should be in dialout group
2. Library auto detects single or double pendulum and safety configuration. Can be changed on your own risk in `robot_single.json` resp. `robot.json` or by passing your own parameter dictionary (same structure as `dict(json.load("robot.json"))`).
