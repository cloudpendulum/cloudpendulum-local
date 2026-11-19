# Cloud Pendulum Local

Driver for local versions of the CloudPendulum single and double pendulums.

## Installation

```
python3 -m venv CPvenv
source CPvenv/bin/activate
pip install -r requirements.txt
echo "import pyCandle" >> $(ls -d CPvenv/lib/python3*)/site-packages/sitecustomize.py
```

Check everything worked
```
lsusb | grep MAB
python3 test.py
```
