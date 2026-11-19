import math
import re
import struct
from dataclasses import dataclass


SYSTEM_STATUS = 0x211
CHASSIS_MOTION = 0x221
STEERING_ANGLES = 0x271
WHEEL_SPEEDS = 0x281
MOTION_MODE = 0x291
CAN_FRAME = struct.Struct('=IB3x8s')

FRAME_FORMATS = {
    SYSTEM_STATUS: struct.Struct('>BBHI'),
    CHASSIS_MOTION: struct.Struct('>hhxxh'),
    STEERING_ANGLES: struct.Struct('>hhhh'),
    WHEEL_SPEEDS: struct.Struct('>hhhh'),
    MOTION_MODE: struct.Struct('>BBB'),
}
CANDUMP_LINE = re.compile(r'^\((\d+(?:\.\d+)?)\)\s+\S+\s+([\da-fA-F]{3})#([\da-fA-F]*)$')


def decode_frame(can_id, payload):
    layout = FRAME_FORMATS.get(can_id)
    if layout is None or len(payload) != layout.size:
        return None

    fields = layout.unpack(payload)
    if can_id == SYSTEM_STATUS:
        status, control_mode, voltage_tenths, faults = fields
        return status, control_mode, voltage_tenths / 10.0, faults
    if can_id in (CHASSIS_MOTION, STEERING_ANGLES, WHEEL_SPEEDS):
        return tuple(value / 1000.0 for value in fields)
    return fields


def replay_frames(lines):
    for line in lines:
        match = CANDUMP_LINE.fullmatch(line.strip())
        if match is None:
            continue
        timestamp, frame_id, data = match.groups()
        can_id = int(frame_id, 16)
        layout = FRAME_FORMATS.get(can_id)
        if layout is not None and len(data) == 2 * layout.size:
            yield float(timestamp), can_id, bytes.fromhex(data)


def body_velocity(mode, motion, wheelbase, track):
    speed, spin_rate, steering = motion
    if mode == 0:
        # Match the upstream dual-Ackermann odometry model using the Delta geometry.
        inner = abs(steering)
        central = math.copysign(math.atan2(
            wheelbase * math.sin(inner),
            wheelbase * math.cos(inner) + track * math.sin(inner),
        ), steering)
        return speed * math.cos(central), 0.0, 2 * speed * math.sin(central) / wheelbase
    if mode == 1:
        return speed * math.cos(steering), speed * math.sin(steering), 0.0
    if mode == 2:
        return 0.0, 0.0, spin_rate
    if mode == 3:
        return 0.0, 0.0, 0.0
    return None


@dataclass
class OdometryEstimate:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    timestamp: float | None = None
    velocity: tuple = (0.0, 0.0, 0.0)

    def update(self, timestamp, velocity, feedback_timeout):
        if self.timestamp is not None:
            dt = timestamp - self.timestamp
            if 0 < dt <= feedback_timeout:
                vx, vy, wz = self.velocity
                heading = self.yaw + wz * dt / 2.0
                self.x += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
                self.y += (vx * math.sin(heading) + vy * math.cos(heading)) * dt
                self.yaw = math.remainder(self.yaw + wz * dt, math.tau)
        self.timestamp = timestamp
        self.velocity = velocity
