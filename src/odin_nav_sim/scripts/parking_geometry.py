"""Shared geometry and reference identity for ODIN parking observations."""
import hashlib
import json
import math


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def pose_xy_yaw(position, quaternion):
    q = quaternion
    return [position.x, position.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                               1 - 2 * (q.y*q.y + q.z*q.z))]


def map_fingerprint(msg):
    info, p, q = msg.info, msg.info.origin.position, msg.info.origin.orientation
    geometry = [msg.header.frame_id, info.width, info.height, info.resolution,
                p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    digest = hashlib.sha256(json.dumps(geometry).encode())
    digest.update(bytes(msg.data))
    return digest.hexdigest() if msg.header.frame_id == 'map' else None
