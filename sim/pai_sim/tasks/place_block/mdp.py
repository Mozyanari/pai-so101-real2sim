"""Termination terms for PlaceBlock."""

from __future__ import annotations

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.envs.mdp import *  # noqa: F401,F403  (template terms such as time_out)
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import quat_apply
from leisaac.tasks.template.mdp import *  # noqa: F401,F403

from so101_pai.teleop.expert import TCP_OFFSET


def block_on_plate_released(
    env: ManagerBasedRLEnv,
    block_cfg: SceneEntityCfg,
    plate_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg,
    xy_margin: float = 0.07,
    height_range: tuple[float, float] = (-0.01, 0.06),
    release_distance: float = 0.05,
) -> torch.Tensor:
    """Success: the block rests within the plate footprint and the gripper has let go and moved away.

    The plate is ~18 cm round, so a 7 cm margin keeps the 4.7 cm block's center inside the rim. "Moved away" is
    measured from the fingertips (TCP): the gripper body origin is 9.3 cm behind them, so measuring from it fired
    while the block was still held over the plate and ended the episode before the release.
    """
    block: RigidObject = env.scene[block_cfg.name]
    plate: RigidObject = env.scene[plate_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    offset = block.data.root_pos_w - plate.data.root_pos_w
    on_plate = (offset[:, 0].abs() < xy_margin) & (offset[:, 1].abs() < xy_margin)
    on_plate &= (offset[:, 2] > height_range[0]) & (offset[:, 2] < height_range[1])

    # Target frame 0 is the "gripper" body.
    offset_b = torch.tensor(TCP_OFFSET, dtype=torch.float32, device=env.device).expand(env.num_envs, 3)
    tcp = ee_frame.data.target_pos_w[:, 0, :] + quat_apply(ee_frame.data.target_quat_w[:, 0, :], offset_b)
    released = torch.linalg.norm(tcp - block.data.root_pos_w, dim=-1) > release_distance
    return on_plate & released


def randomize_block_color(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    base: tuple[float, float, float],
    brightness: tuple[float, float],
    jitter: float,
) -> None:
    """Reset event: tint the block's PreviewSurface (spawned via visual_material) around `base`.

    Each reset scales `base` by a brightness factor and adds a small per-channel jitter, so the color stays wood-like
    instead of an arbitrary uniform RGB sample. Edits the USD shader, which the renderer picks up on the next frame.
    """
    from isaaclab.sim.utils import find_matching_prims
    from pxr import Gf

    asset: RigidObject = env.scene[asset_cfg.name]
    for shader in find_matching_prims(f"{asset.cfg.prim_path}/material/Shader"):
        scale = torch.empty(1).uniform_(*brightness).item()
        color = [min(max(c * scale + torch.empty(1).uniform_(-jitter, jitter).item(), 0.0), 1.0) for c in base]
        shader.GetAttribute("inputs:diffuseColor").Set(Gf.Vec3f(*color))
