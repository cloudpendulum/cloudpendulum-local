# Cloud Pendulum Local

Driver for local versions of the CloudPendulum single and double pendulums.

## Installation

```
# Create venv and activate it
python3 -m venv CPvenv
source CPvenv/bin/activate
# Install requirements
pip install -r requirements.txt
# Work around C++ library linking bug in MAB motor driver
echo "import pyCandle" >> $(ls -d CPvenv/lib/python3*)/site-packages/sitecustomize.py
```

Check everything worked
```
lsusb | grep MAB
python3 test.py SimplePendulum # resp. DoublePendulum
```
If your drive is still not recognized, it might be connected to the baud rate the motor driver is configured to. This can be changed permamently using MAB Robotics tools (see their website), or you connect with the expected rate:
```
python3 test.py SimplePendulum --baud-rate 1M # or 2M,5M,8M
```


Note: Your user should be in dialout group
