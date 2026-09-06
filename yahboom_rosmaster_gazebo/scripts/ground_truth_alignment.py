#!/usr/bin/env python3
"""Small rigid-transform helpers for ground-truth frame alignment."""

import math
from typing import NamedTuple


class RigidTransform(NamedTuple):
    """Translation and quaternion for one parent-to-child transform."""

    translation: tuple
    rotation: tuple


def normalize_quaternion(quaternion):
    """Return a finite unit quaternion in ROS x/y/z/w order."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must be finite and non-zero")
    return tuple(value / norm for value in quaternion)


def multiply_quaternions(left, right):
    """Compose two ROS-order quaternions."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_vector(quaternion, vector):
    """Rotate a three-vector by a ROS-order quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    # Equivalent to q * (v, 0) * inverse(q), expanded to avoid allocating
    # temporary quaternions whose vector form is intentionally non-unit.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose(parent_to_middle, middle_to_child):
    """Return parent-to-child from parent-to-middle and middle-to-child."""
    rotated = rotate_vector(
        parent_to_middle.rotation, middle_to_child.translation)
    return RigidTransform(
        tuple(
            parent + child
            for parent, child in zip(parent_to_middle.translation, rotated)
        ),
        multiply_quaternions(
            parent_to_middle.rotation, middle_to_child.rotation),
    )


def inverse(transform):
    """Return the inverse of a rigid transform."""
    qx, qy, qz, qw = normalize_quaternion(transform.rotation)
    inverse_rotation = (-qx, -qy, -qz, qw)
    inverse_translation = rotate_vector(
        inverse_rotation,
        tuple(-value for value in transform.translation),
    )
    return RigidTransform(inverse_translation, inverse_rotation)


def yaw_quaternion(yaw):
    """Return a quaternion for a pure yaw rotation (mainly for tests)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
