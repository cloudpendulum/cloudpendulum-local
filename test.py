from cloud_pendulum_local import Client
import math
import time

client = Client()
client.start_experiment()

experiment_type = "SimplePendulum"
Tf = 5.0
kp = 0.20
kd = 0.01
if experiment_type == "DoublePendulum":
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

    if experiment_type == "DoublePendulum":
        client.set_position([meas_time * 4.0] * 2)
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
