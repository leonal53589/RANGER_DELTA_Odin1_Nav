import math
import struct
from dataclasses import dataclass


ACKERMANN = 0
CRAB = 1
SPIN = 2
STANDBY = 0
CAN_CONTROL = 1
SET_CONTROL = 0x421
SET_MOTION_MODE = 0x141
SET_MOTION = 0x111
STOP_FRAME = (SET_MOTION, bytes(8))


@dataclass(frozen=True)
class MotionCommand:
    mode: int
    speed: float = 0.0
    yaw_rate: float = 0.0
    steering: float = 0.0

    def frame(self):
        return SET_MOTION, struct.pack(
            '>hhxxh', round(self.speed * 1000), round(self.yaw_rate * 1000), round(self.steering * 1000),
        )


def velocity_command(vx, vy, wz, *, wheelbase, track, max_linear, max_angular, min_turn_radius):
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        return None
    vx, vy, wz = (value if abs(value) >= 0.001 else 0.0 for value in (vx, vy, wz))
    if vx == vy == wz == 0.0 or (vy and wz):
        # The documented crab mode translates with a fixed heading.
        return None
    wz = math.copysign(min(abs(wz), max_angular, 3.259), wz)
    if vx == vy == 0.0:
        return MotionCommand(SPIN, yaw_rate=wz)

    if vy:
        if vx == 0.0:
            # Ranger Delta physical steering range is [-30°, +90°].
            # In CAN 0x111, negative values steer the wheels to positive angles [0, +90°].
            # Positive values (> 0.52 rad) hit the -30° negative physical stop.
            # To support lateral crab motion in both directions (left and right),
            # the wheels are aligned horizontally at CAN steering -1.571 rad (+90° physical).
            # Lateral direction is controlled by reversing wheel drive velocity:
            # vy < 0 (Right): speed > 0, steering = -pi/2
            # vy > 0 (Left):  speed < 0, steering = -pi/2
            steering = -math.pi / 2
            direction = -1.0 if vy > 0.0 else 1.0
        else:
            direction = -1.0 if vx < 0.0 else 1.0
            steering = math.atan2(direction * vy, direction * vx)
            if steering > 0.523:
                steering -= math.pi
                direction = -direction
            elif steering < -math.pi / 2:
                steering = -math.pi / 2
        mode = CRAB
    else:
        direction = math.copysign(1.0, vx)
        steering = 0.0
        if wz:
            radius = max(abs(vx / wz), min_turn_radius)
            steering = math.copysign(min(math.atan2(wheelbase / 2, radius - track / 2), 0.698), vx * wz)
        mode = ACKERMANN

    limit = min(max_linear, 0.525 if abs(steering) > math.radians(20) else 1.5)
    return MotionCommand(mode, speed=direction * min(math.hypot(vx, vy), limit), steering=steering)


class CommandGate:
    def __init__(self, command_timeout, feedback_timeout):
        self.command_timeout = command_timeout
        self.feedback_timeout = feedback_timeout
        self.active_mode = None
        self.last_request = None
        self.last_request_time = -math.inf

    def stop(self):
        if self.active_mode is None:
            return ()
        self.active_mode = None
        return (STOP_FRAME,)

    def frames(self, now, command, status, mode):
        if command is None or status is None or mode is None:
            return self.stop()
        command_time, request = command
        status_time, status_code, control_mode, has_faults = status
        mode_time, motion_mode, switching = mode
        if (request is None or status_code != 0 or has_faults or switching
                or control_mode not in (STANDBY, CAN_CONTROL)
                or now - command_time > self.command_timeout
                or now - min(status_time, mode_time) > self.feedback_timeout):
            return self.stop()

        if self.active_mode is not None and (self.active_mode != request.mode
                                             or motion_mode != request.mode or control_mode != CAN_CONTROL):
            return self.stop()
        if control_mode == STANDBY:
            return self.request(now, (SET_CONTROL, bytes([CAN_CONTROL])))
        if self.last_request is not None and self.last_request[0] == SET_MOTION_MODE:
            pending_mode = self.last_request[1][0]
            if motion_mode != pending_mode or mode_time <= self.last_request_time:
                # Wait for the pending switch before acting on a newer velocity request.
                if request.mode == pending_mode or now - self.last_request_time >= 0.5:
                    return self.request(now, (SET_MOTION_MODE, bytes([request.mode])))
                return ()
            self.last_request = None
        if motion_mode != request.mode:
            return self.request(now, (SET_MOTION_MODE, bytes([request.mode])))
        if self.last_request is not None and min(status_time, mode_time) <= self.last_request_time:
            return ()

        self.last_request = None
        self.active_mode = request.mode
        return (request.frame(),)

    def request(self, now, frame):
        if now - self.last_request_time < 0.1:
            return ()
        self.last_request = frame
        self.last_request_time = now
        return (frame,)
