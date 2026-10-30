"""Scripted pick-and-place expert: stands in for the human on the leader arm when collecting sim demos.

It plans a top-down grasp of the block and a drop over the plate as timed waypoints of the tool center point (TCP,
between the fingertips), and the sim bridge (`sim/ros_bridge.py --expert`) turns them into joint commands with
damped least squares on the sim's Jacobian, using the helpers here. Plain numpy, no Isaac or ROS.

Frames: the env frame (robot base at the origin, table top at z=0, meters). Rotations are 3x3 matrices whose
columns are the gripper body's axes; the gripper points along its -z axis and the jaw opens along its x axis.
The SO-101 has one fixed finger (part of the gripper body) and one moving jaw, and the TCP is on the fixed finger's
inner face, where the jaw closes onto it. So a grasped block's center sits half a block along +x from the TCP.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# TCP in the gripper body frame: between the closed fingertips (measured in the sim, 9.3 cm along -z).
TCP_OFFSET = np.array([0.0, 0.0, -0.093])
GRIPPER_OPEN = 1.0  # rad; opening to 1.2 rad grasped no better
# The jaw touches a 4.7 cm block at ~24 deg. Commanding the joint's lower limit (-10 deg) made the soft position
# drive push with its full 10 Nm (>100 N at the pads) and the block squirted out, so close only ~7 deg past contact.
GRIPPER_CLOSED = 0.30  # rad
BLOCK_HALF = 0.047 / 2
PLATE_TOP = 0.018  # the scaled plate is 1.8 cm tall
CARRY_HEIGHT = 0.12  # TCP height while moving to the plate
# Gap between the fixed finger and the block face while descending; closing the jaw then slides the block over.
# 1.2 cm would leave a learned policy more margin, but then the expert's own grasps mostly failed (6/20).
FINGER_CLEARANCE = 0.006


@dataclass(frozen=True)
class Waypoint:
    """Reach `pos`/`rot` with the gripper joint at `gripper` by the end of a `duration` s segment."""

    pos: np.ndarray
    rot: np.ndarray
    gripper: float
    duration: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def jaw_heading(block_yaw: float, radial: float) -> float:
    """Direction (yaw) for the jaw axis: the block face normal closest to the arm's radial direction.

    Squeezing across two opposite faces needs the jaw axis along a face normal (block_yaw + k * 90 deg); the one
    nearest the radial direction keeps the wrist roll small.
    """
    k = round(wrap_angle(radial - block_yaw) / (math.pi / 2))
    return wrap_angle(block_yaw + k * math.pi / 2)


def top_down_rotation(heading: float) -> np.ndarray:
    """Gripper pointing straight down (-z axis = world -Z) with its jaw axis (x) at yaw `heading`."""
    x = np.array([math.cos(heading), math.sin(heading), 0.0])
    z = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def plan_pick_place(
    block_pos: np.ndarray,
    block_yaw: float,
    plate_pos: np.ndarray,
    rng: np.random.Generator | None = None,
) -> list[Waypoint]:
    """Waypoints from the current pose to the block, onto the plate, and away.

    rng varies every episode for data diversity, within what still grasps reliably: overall tempo, grasp point and
    heading, approach/carry/retreat heights, a sideways detour on the way to the plate, and the drop point.
    """
    rng = rng or np.random.default_rng()
    block_pos, plate_pos = np.asarray(block_pos, dtype=float), np.asarray(plate_pos, dtype=float)
    tempo = rng.uniform(0.8, 1.25)  # > 1 is faster
    seconds = lambda base: base * rng.uniform(0.85, 1.15) / tempo  # noqa: E731

    block_radial = math.atan2(block_pos[1], block_pos[0])
    plate_radial = math.atan2(plate_pos[1], plate_pos[0])
    heading = jaw_heading(block_yaw, block_radial) + math.radians(rng.uniform(-5.0, 5.0))
    grasp_rot = top_down_rotation(heading)
    # Keep the wrist roll while the shoulder pans to the plate: turn the jaw axis by the same angle.
    place_rot = top_down_rotation(heading + wrap_angle(plate_radial - block_radial))

    up = np.array([0.0, 0.0, 1.0])
    # The fixed finger goes down beside the block; closing the jaw then pushes the block against it.
    grasp = np.array([block_pos[0], block_pos[1], BLOCK_HALF]) - (BLOCK_HALF + FINGER_CLEARANCE) * grasp_rot[:, 0]
    grasp[:2] += rng.uniform(-0.004, 0.004, size=2)
    drop = np.array([*(plate_pos[:2] + rng.uniform(-0.02, 0.02, size=2)), PLATE_TOP + BLOCK_HALF + 0.01])
    place = drop - BLOCK_HALF * place_rot[:, 0]  # where the TCP is when the held block's center is at `drop`
    carry = CARRY_HEIGHT + rng.uniform(-0.02, 0.03)
    # Detour: halfway to the plate, off the straight line by up to 3 cm sideways and 2 cm up.
    side = np.cross(up, np.append(place[:2] - grasp[:2], 0.0))
    side /= max(np.linalg.norm(side), 1e-9)
    midway = np.array([*(grasp[:2] + place[:2]) / 2, carry]) + rng.uniform(-0.03, 0.03) * side
    midway[2] += rng.uniform(0.0, 0.02)
    mid_rot = interpolate_rotation(grasp_rot, place_rot, 0.5)
    retreat = place + rng.uniform(0.06, 0.10) * up + np.append(rng.uniform(-0.02, 0.02, size=2), 0.0)

    open_, closed = GRIPPER_OPEN, GRIPPER_CLOSED
    return [
        Waypoint(grasp + rng.uniform(0.06, 0.10) * up, grasp_rot, open_, seconds(2.0)),
        Waypoint(grasp, grasp_rot, open_, seconds(1.0)),
        Waypoint(grasp, grasp_rot, closed, 0.6),
        Waypoint(np.array([*grasp[:2], carry]), grasp_rot, closed, seconds(1.0)),
        Waypoint(midway, mid_rot, closed, seconds(0.9)),
        Waypoint(np.array([*place[:2], carry]), place_rot, closed, seconds(0.9)),
        Waypoint(place, place_rot, closed, seconds(0.8)),
        Waypoint(place, place_rot, open_, 0.5),
        Waypoint(retreat, place_rot, open_, seconds(0.8)),
    ]


class SmoothNoise:
    """Ornstein-Uhlenbeck noise: zero mean, stationary std `sigma` per component, correlation time `tau` (s).

    The sim bridge adds it to the expert's joint commands before executing them but records the commands without
    it (DART), so the demos show the expert pulling back from states slightly off its own path.
    """

    def __init__(self, sigma: np.ndarray, tau: float, dt: float, rng: np.random.Generator) -> None:
        self.sigma = np.asarray(sigma, dtype=float)
        self.decay = math.exp(-dt / tau)
        self.rng = rng
        self.value = np.zeros_like(self.sigma)

    def reset(self) -> None:
        self.value = np.zeros_like(self.sigma)

    def step(self) -> np.ndarray:
        shock = self.rng.normal(size=self.sigma.shape) * self.sigma * math.sqrt(1 - self.decay**2)
        self.value = self.decay * self.value + shock
        return self.value


def plan_duration(plan: list[Waypoint]) -> float:
    return sum(w.duration for w in plan)


def sample(plan: list[Waypoint], start: Waypoint, t: float) -> Waypoint:
    """The commanded TCP pose and gripper at time t: smoothstep from each waypoint to the next."""
    previous = start
    for waypoint in plan:
        if t < waypoint.duration:
            s = t / waypoint.duration
            s = s * s * (3 - 2 * s)
            return Waypoint(
                previous.pos + s * (waypoint.pos - previous.pos),
                interpolate_rotation(previous.rot, waypoint.rot, s),
                previous.gripper + s * (waypoint.gripper - previous.gripper),
                0.0,
            )
        t -= waypoint.duration
        previous = waypoint
    return Waypoint(previous.pos, previous.rot, previous.gripper, 0.0)


def rotation_vector(rot: np.ndarray) -> np.ndarray:
    """Axis * angle of a rotation matrix (log map)."""
    cos = np.clip((np.trace(rot) - 1) / 2, -1.0, 1.0)
    angle = math.acos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:  # 180 deg: (R + I) / 2 = axis axis^T, so any non-zero column is the axis
        outer = (rot + np.eye(3)) / 2
        axis = outer[:, np.argmax(np.diag(outer))]
        return angle * axis / np.linalg.norm(axis)
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) / (2 * math.sin(angle))
    return angle * axis


def rotation_from_vector(vec: np.ndarray) -> np.ndarray:
    """Exponential map (Rodrigues)."""
    angle = float(np.linalg.norm(vec))
    if angle < 1e-12:
        return np.eye(3)
    k = np.asarray(vec) / angle
    kx = skew(k)
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def interpolate_rotation(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    return rotation_from_vector(s * rotation_vector(b @ a.T)) @ a


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def pose_error(pos: np.ndarray, rot: np.ndarray, target_pos: np.ndarray, target_rot: np.ndarray) -> np.ndarray:
    """6-vector (position error, rotation error as a world-frame rotation vector) to go from pose to target."""
    return np.concatenate([target_pos - pos, rotation_vector(target_rot @ rot.T)])


def tcp_jacobian(jacobian: np.ndarray, tcp_from_body_world: np.ndarray) -> np.ndarray:
    """Jacobian of a point offset from the body origin by `tcp_from_body_world` (world frame), [linear; angular]."""
    linear = jacobian[:3] - skew(tcp_from_body_world) @ jacobian[3:]
    return np.vstack([linear, jacobian[3:]])


def damped_least_squares(jacobian: np.ndarray, error: np.ndarray, damping: float = 0.05) -> np.ndarray:
    """Joint step that best reduces `error`: J^T (J J^T + damping^2 I)^-1 error."""
    jjt = jacobian @ jacobian.T
    return jacobian.T @ np.linalg.solve(jjt + damping**2 * np.eye(len(jjt)), error)
