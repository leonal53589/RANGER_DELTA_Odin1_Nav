import math
import struct

from ranger_delta_bringup.commands import (
    ACKERMANN,
    CAN_CONTROL,
    CRAB,
    SET_CONTROL,
    SET_MOTION,
    SET_MOTION_MODE,
    SPIN,
    STANDBY,
    STOP_FRAME,
    CommandGate,
    MotionCommand,
    velocity_command,
)


LIMITS = dict(wheelbase=0.378, track=0.338, max_linear=0.35, max_angular=0.6, min_turn_radius=0.5)


def test_velocity_encoding_and_limits():
    straight = velocity_command(0.7, 0.0, 0.0, **LIMITS)
    assert straight.mode == ACKERMANN
    assert straight.frame() == (SET_MOTION, bytes.fromhex('015E000000000000'))

    reverse_turn = velocity_command(-0.2, 0.0, 0.6, **LIMITS)
    assert reverse_turn.mode == ACKERMANN and reverse_turn.steering < 0
    assert struct.unpack('>hhxxh', reverse_turn.frame()[1])[:2] == (-200, 0)

    crab = velocity_command(0.0, -0.2, 0.0, **LIMITS)
    assert crab.mode == CRAB
    assert struct.unpack('>hhxxh', crab.frame()[1]) == (200, 0, -1571)

    crab_left = velocity_command(0.0, 0.2, 0.0, **LIMITS)
    assert crab_left.mode == CRAB
    assert struct.unpack('>hhxxh', crab_left.frame()[1]) == (-200, 0, -1571)

    spin = velocity_command(0.0, 0.0, -1.0, **LIMITS)
    assert spin.mode == SPIN
    assert struct.unpack('>hhxxh', spin.frame()[1]) == (0, -600, 0)


def test_unsupported_or_invalid_twist_does_not_request_motion():
    assert velocity_command(0.0, 0.0, 0.0, **LIMITS) is None
    assert velocity_command(0.0, 0.1, 0.1, **LIMITS) is None
    assert velocity_command(math.nan, 0.0, 0.0, **LIMITS) is None


def test_startup_and_standby_wait_for_chassis_acknowledgement():
    gate = CommandGate(0.30, 0.20)
    drive = MotionCommand(ACKERMANN, 0.2)
    standby = (1.0, 0, STANDBY, False)
    mode = (1.0, ACKERMANN, 0)
    assert gate.frames(1.0, None, standby, mode) == ()
    assert gate.frames(1.0, (1.0, drive), standby, mode) == ((SET_CONTROL, b'\x01'),)
    assert gate.frames(1.04, (1.0, drive), standby, mode) == ()
    assert gate.frames(1.05, (1.0, drive), (1.05, 0, CAN_CONTROL, False), mode) == ()
    assert gate.frames(1.06, (1.0, drive), (1.05, 0, CAN_CONTROL, False), (1.06, ACKERMANN, 0)) == (drive.frame(),)


def test_switching_stops_and_waits_for_new_mode_feedback():
    gate = CommandGate(0.30, 0.20)
    drive = MotionCommand(ACKERMANN, 0.2)
    spin = MotionCommand(SPIN, yaw_rate=-0.4)
    status = (1.0, 0, CAN_CONTROL, False)
    mode = (1.0, ACKERMANN, 0)
    assert gate.frames(1.0, (1.0, drive), status, mode) == (drive.frame(),)
    assert gate.frames(1.01, (1.01, spin), status, mode) == (STOP_FRAME,)
    assert gate.frames(1.02, (1.01, spin), status, mode) == ((SET_MOTION_MODE, b'\x02'),)
    assert gate.frames(1.04, (1.04, drive), (1.03, 0, CAN_CONTROL, False), (1.03, ACKERMANN, 0)) == ()
    assert gate.frames(1.05, (1.05, spin), status, (1.05, SPIN, 1)) == ()
    assert gate.frames(1.06, (1.05, spin), (1.05, 0, CAN_CONTROL, False), (1.06, SPIN, 0)) == (spin.frame(),)


def test_stale_feedback_stale_velocity_and_remote_override_stop():
    drive = MotionCommand(ACKERMANN, 0.2)
    faults_or_stale = [
        ((1.30, 0, CAN_CONTROL, False), (1.0, ACKERMANN, 0), (1.3, drive)),
        ((1.31, 0, CAN_CONTROL, False), (1.31, ACKERMANN, 0), (1.0, drive)),
        ((1.31, 0, 3, False), (1.31, ACKERMANN, 0), (1.3, drive)),
        ((1.31, 0, CAN_CONTROL, True), (1.31, ACKERMANN, 0), (1.3, drive)),
        ((1.31, 0, CAN_CONTROL, False), (1.31, ACKERMANN, 0), (1.3, None)),
    ]
    for status, mode, command in faults_or_stale:
        gate = CommandGate(0.30, 0.20)
        assert gate.frames(1.0, (1.0, drive), (1.0, 0, CAN_CONTROL, False), (1.0, ACKERMANN, 0)) == (drive.frame(),)
        assert gate.frames(1.31, command, status, mode) == (STOP_FRAME,)
        assert gate.frames(1.32, command, status, mode) == ()
