"""Limit localization corrections while the Ranger Delta spins in place."""

import math
from dataclasses import dataclass

import PyKDL


@dataclass(frozen=True)
class ChassisMotion:
    mode: int
    switching: int
    linear_speed: float
    yaw_rate: float


class SpinAlignmentGuard:
    """Propagate wheel motion through a spin and recover against ODIN while driving."""

    def __init__(self):
        self.previous = None
        self.protecting = False
        self.state = 'tracking'

    def update(self, stamp_ns, alignment, odom_from_base, motion):
        raw_base = alignment * odom_from_base
        output = raw_base
        self.state = 'tracking'
        if self.previous is not None:
            last_stamp, last_base, last_wheels = self.previous
            elapsed = (stamp_ns - last_stamp) / 1e9
            step = last_wheels.Inverse() * odom_from_base
            step_yaw = step.M.GetRPY()[2]
            if not 0 < elapsed <= 0.50 or step.p.Norm() > 0.12 or abs(step_yaw) > 0.20:
                # Never propagate through a reset of the wheel odometry or a long TF gap.
                self.protecting = False
            else:
                if (motion is not None and motion.mode == 2 and motion.switching == 0
                        and motion.linear_speed <= 0.015 and abs(motion.yaw_rate) >= 0.025):
                    self.protecting = True
                if self.protecting:
                    predicted = last_base * step
                    correction = raw_base.p - predicted.p
                    yaw_error = math.remainder(raw_base.M.GetRPY()[2] - predicted.M.GetRPY()[2], math.tau)
                    translating = (motion is not None and motion.mode in (0, 1)
                                   and motion.switching == 0 and motion.linear_speed >= 0.018
                                   and elapsed <= 0.25)
                    if translating:
                        self.state = 'recovering'
                        distance = step.p.Norm()
                        xy_limit = min(0.025 * elapsed, 0.125 * distance)
                        yaw_limit = min(0.10 * elapsed, 0.30 * distance)
                    else:
                        self.state = 'holding'
                        xy_limit = 0.0
                        yaw_limit = 0.04 * elapsed if elapsed <= 0.25 else 0.0

                    distance_error = correction.Norm()
                    fraction = min(1.0, xy_limit / distance_error) if distance_error else 1.0
                    yaw_step = math.copysign(min(abs(yaw_error), yaw_limit), yaw_error)
                    output = PyKDL.Frame(
                        PyKDL.Rotation.RotZ(predicted.M.GetRPY()[2] + yaw_step),
                        predicted.p + correction * fraction,
                    )
                    if translating and fraction == 1.0 and abs(yaw_error) <= yaw_limit:
                        self.protecting = False

        self.previous = stamp_ns, output, odom_from_base
        return output * odom_from_base.Inverse() if self.state != 'tracking' else alignment
