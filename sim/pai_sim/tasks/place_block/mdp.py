"""Termination terms for PlaceBlock."""

from __future__ import annotations

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.envs.mdp import *  # noqa: F401,F403  (template terms such as time_out)
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from leisaac.tasks.template.mdp import *  # noqa: F401,F403


def block_on_plate_released(
    env: ManagerBasedRLEnv,
    block_cfg: SceneEntityCfg,
    plate_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg,
    xy_margin: float = 0.07,
    height_range: tuple[float, float] = (-0.01, 0.06),
    release_distance: float = 0.08,
) -> torch.Tensor:
    """Success: the block rests within the plate footprint and the gripper has let go and moved away.

    The plate is 17 cm square, so a 7 cm margin keeps the 4.7 cm block's center inside the rim.
    """
    block: RigidObject = env.scene[block_cfg.name]
    plate: RigidObject = env.scene[plate_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    offset = block.data.root_pos_w - plate.data.root_pos_w
    on_plate = (offset[:, 0].abs() < xy_margin) & (offset[:, 1].abs() < xy_margin)
    on_plate &= (offset[:, 2] > height_range[0]) & (offset[:, 2] < height_range[1])

    gripper_pos = ee_frame.data.target_pos_w[:, 0, :]  # target frame 0 is "gripper"
    released = torch.linalg.norm(gripper_pos - block.data.root_pos_w, dim=-1) > release_distance
    return on_plate & released
