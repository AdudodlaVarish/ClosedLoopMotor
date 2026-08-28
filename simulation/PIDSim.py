import argparse
import numpy as np
import matplotlib.pyplot as plt

def parse_clargs():
    parser = argparse.ArgumentParser()

    # Motor Physics
    parser.add_argument("--J", type=float, default=0.01,
                         help="Rotational inertia (default: 0.01)")
    parser.add_argument("--b", type=float, default=0.08,
                         help="Friction coefficient (default: 0.08)")
    parser.add_argument("--motor-k", type=float, default=0.8,
                         help="Motor strength / gain (default: 0.8)")

    # PID Gains
    parser.add_argument("--Kp", type=float, default=2.0,
                         help="Proportional gain (default: 2.0)")
    parser.add_argument("--Ki", type=float, default=1.5,
                         help="Integral gain (default: 1.5)")
    parser.add_argument("--Kd", type=float, default=0.08,
                         help="Derivative gain (default: 0.08)")

    # Disturbance
    parser.add_argument("--load-torque", type=float, default=0.25,
                         help="Magnitude of disturbance load torque (default: 0.25)")

    return parser.parse_args()

def run_simulation(args):

    dt = 0.002
    duration = 4.0
    time = np.arange(0, duration, dt)

    J = args.J       # rotational inertia
    b = args.b      # friction
    motor_k = args.motor_k  # motor strength

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
            args.Kp * error
            + args.Ki * integral
            + args.Kd * derivative
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

    return time, targets, positions, speeds, outputs

def plot(time, targets, positions, speeds, outputs):

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

if __name__ == "__main__":
    args = parse_clargs()
    time, targets, positions, speeds, outputs = run_simulation(args)
    plot(time, targets, positions, speeds, outputs)
