"""Fit a yaw plant from a SysId Yaw Log and calculate PD gains with WPIMath."""

import argparse
from pathlib import Path
import re

import numpy as np
from wpimath.controller import LinearQuadraticRegulator_2_1
from wpimath.system import LinearSystem_2_1_1


def read_capture(path):
    samples = None
    complete = False
    for line in path.read_text(errors="replace").splitlines():
        if "SYSID_BEGIN" in line:
            samples = []
            complete = False
        elif "SYSID_CANCEL" in line:
            samples = None
            complete = False
        elif "SYSID_END" in line:
            complete = samples is not None
        elif samples is not None and not complete:
            match = re.search(r"SYSID,(\d+),(-?\d+),(-?\d+)", line)
            if match:
                samples.append(tuple(map(int, match.groups())))

    if not complete or len(samples) < 50:
        raise ValueError("No complete SysId capture with at least 50 samples")

    micros, command, yaw = np.asarray(samples, dtype=float).T
    time = (micros - micros[0]) * 1e-6
    intervals = np.diff(time)
    if np.any(intervals <= 0) or np.max(intervals) > 0.1:
        raise ValueError("Capture has out-of-order timestamps or a gap over 100 ms")
    return time, command, np.rad2deg(np.unwrap(np.deg2rad(yaw)))


def fit_plant(time, command, angle):
    # Integrate u = kS sign(rate) + kV rate + kA acceleration over short
    # windows. Position differences avoid taking two derivatives of integer yaw.
    half_velocity_window = 0.035
    fit_window = 0.08
    start = np.arange(
        time[0] + half_velocity_window,
        time[-1] - half_velocity_window - fit_window,
        0.02,
    )
    end = start + fit_window

    def velocity_at(t):
        return (
            np.interp(t + half_velocity_window, time, angle)
            - np.interp(t - half_velocity_window, time, angle)
        ) / (2 * half_velocity_window)

    start_rate, end_rate = velocity_at(start), velocity_at(end)
    movement_sign = np.sign(velocity_at((start + end) / 2))
    moving = (start_rate * movement_sign >= 5) & (end_rate * movement_sign >= 5)
    if np.count_nonzero(moving) < 20 or np.unique(movement_sign[moving]).size < 2:
        raise ValueError("Insufficient measurable rotation in both directions")

    # Each command is held until the next sample. Integrate it as a step signal.
    command_area = np.r_[0, np.cumsum(command[:-1] * np.diff(time))]

    def integrated_command(t):
        index = np.searchsorted(time, t, side="right") - 1
        return command_area[index] + command[index] * (t - time[index])

    features = np.column_stack((
        movement_sign * fit_window,
        np.interp(end, time, angle) - np.interp(start, time, angle),
        end_rate - start_rate,
    ))[moving]
    response = (integrated_command(end) - integrated_command(start))[moving]
    gains, _, rank, _ = np.linalg.lstsq(features, response, rcond=None)
    residual = response - features @ gains
    variation = np.sum((response - np.mean(response)) ** 2)
    if rank != 3 or variation <= 0:
        raise ValueError("Capture does not identify all three plant coefficients")
    r_squared = 1 - np.sum(residual**2) / variation
    ks, kv, ka = gains
    if r_squared < 0.7 or ks < 0 or kv <= 0 or ka <= 0:
        raise ValueError(
            f"Unreliable fit: kS={ks:.3g}, kV={kv:.3g}, kA={ka:.3g}, R²={r_squared:.3f}"
        )
    return ks, kv, ka, r_squared


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="UART log from Debug menu > SysId Yaw Log")
    parser.add_argument("--dt", type=float, default=0.01, help="Control period in seconds")
    parser.add_argument("--yaw-error", type=float, default=8.0, help="LQR angle tolerance in degrees")
    parser.add_argument("--rate-error", type=float, default=45.0, help="LQR rate tolerance in degrees/s")
    parser.add_argument("--effort", type=float, default=20.0, help="LQR effort in signed motor command units")
    args = parser.parse_args()
    if any(not np.isfinite(value) or value <= 0 for value in
           (args.dt, args.yaw_error, args.rate_error, args.effort)):
        parser.error("Controller settings must be finite and positive")

    try:
        ks, kv, ka, r_squared = fit_plant(*read_capture(args.capture))
        plant = LinearSystem_2_1_1(
            [[0, 1], [0, -kv / ka]], [[0], [1 / ka]], [[1, 0]], [[0]]
        )
        lqr = LinearQuadraticRegulator_2_1(
            plant, (args.yaw_error, args.rate_error), (args.effort,), args.dt
        )
        kp, kd = lqr.K(0, 0), lqr.K(0, 1)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f"SysId: {error}\n")

    print(f"Model: kS={ks:.4g}, kV={kv:.4g}, kA={ka:.4g}, R²={r_squared:.3f}")
    print(f"Starting PD gains: Kp={kp:.5g}, Ki=0, Kd={kd:.5g}")


if __name__ == "__main__":
    main()
