import numpy as np
import matplotlib.pyplot as plt

dt = 0.002
duration = 4.0
time = np.arange(0, duration, dt)

J = 0.01       # rotational inertia
b = 0.08       # friction
motor_k = 0.8  # motor strength

Kp = 2.0
Ki = 1.5
Kd = 0.08

# Simulated state
position = 0.0
speed = 0.0
integral = 0.0
previous_error = 0.0

targets = []
positions = []
speeds = []
outputs = []

for t in time:
    target = np.deg2rad(90) if t >= 0.5 else 0.0

    measured_position = position
    measured_speed = speed

    error = target - measured_position
    derivative = (error - previous_error) / dt

    raw_output = (
        Kp * error
        + Ki * integral
        + Kd * derivative
    )

    output = np.clip(raw_output, -1.0, 1.0)

    # Only add to integral when necessary, ex + error (target > measured) and (-) output (position decreasing)
    if abs(raw_output) < 1.0 or np.sign(error) != np.sign(raw_output):
        integral += error * dt

    # Simulate someone pushing against the arm (0.3s)
    load_torque = 0.25 if 2.0 <= t < 2.3 else 0.0

    # Motor physics
    acceleration = (
        motor_k * output
        - b * measured_speed
        - load_torque
    ) / J

    speed += acceleration * dt
    position += speed * dt
    previous_error = error

    targets.append(np.rad2deg(target))
    positions.append(np.rad2deg(position))
    speeds.append(np.rad2deg(speed))
    outputs.append(output * 100)

fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axes[0].plot(time, targets, "--", label="Target")
axes[0].plot(time, positions, label="Position")
axes[0].set_ylabel("Degrees")
axes[0].legend()
axes[0].grid()

axes[1].plot(time, speeds)
axes[1].set_ylabel("Speed (deg/s)")
axes[1].grid()

axes[2].plot(time, outputs)
axes[2].set_ylabel("Motor command (%)")
axes[2].set_xlabel("Time (seconds)")
axes[2].set_ylim(-110, 110)
axes[2].grid()

plt.tight_layout()
plt.show()