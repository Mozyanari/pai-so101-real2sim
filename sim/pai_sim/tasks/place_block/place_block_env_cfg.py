"""PlaceBlock: pick a 4.7 cm block and place it on a white plate with the SO-101.

Built on LeIsaac's single-arm template (Apache-2.0). Assets are NVIDIA-official:
Isaac Sim 5.1 Props/Blocks/red_block.usd and SimReady dishware_plastic_smallPlate_a01 / dishware_bowl_a01
(all fetched by setup/04_task_assets.sh).
The plate is rescaled and recolored white to match a real ~18 cm white paper plate.
Env overrides, to match the real objects: PAI_PLATE (square | round), PAI_BLOCK_SIZE (m), PAI_BLOCK_MASS (kg),
PAI_BLOCK_COLOR (red: the Isaac red block's texture | wood: beech, re-tinted every reset), and PAI_LAYOUT
(default | real1031: plate, block area, front camera and robot color as measured on the real setup on 10/31).
The table is a plain collision box whose top is at z=0; the photoreal background (3DGS) is layered on later.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RecorderManagerBaseCfg, SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from leisaac.tasks.template import (
    SingleArmObservationsCfg,
    SingleArmTaskEnvCfg,
    SingleArmTaskSceneCfg,
    SingleArmTerminationsCfg,
)
from leisaac.utils.domain_randomization import (
    domain_randomization,
    randomize_camera_uniform,
    randomize_object_uniform,
)
from pxr import Usd

from . import mdp

ASSETS = Path(__file__).resolve().parents[4] / "assets" / "pai"
BLOCK_USD = str(ASSETS / "blocks" / "red_block.usd")
PLATE_USD = str(
    ASSETS / "dishware_plastic_smallPlate_a01" / "simready_usd" / "sm_dishware_plastic_smallPlate_a01_01.usd"
)
# The asset is a 9.9 cm x 4.5 cm deep plate with a rounded-square rim (11.3 cm across the corners), not round;
# flatten and widen it to ~17.8 cm (20.4 cm across the corners) x 1.8 cm, about an 18 cm paper plate.
SQUARE_PLATE = (PLATE_USD, (1.8, 1.8, 0.4))
# The real paper plate is round (decided 10/31). SimReady has no round plate, so flatten its round bowl
# (16.3 cm x 5.5 cm) to 18 cm x 1.8 cm.
BOWL_USD = str(ASSETS / "dishware_bowl_a01" / "simready_usd" / "sm_dishware_bowl_a01_01.usda")
ROUND_PLATE = (BOWL_USD, (1.106, 1.106, 0.33))
PLATE_USD, PLATE_SCALE = {"square": SQUARE_PLATE, "round": ROUND_PLATE}[os.environ.get("PAI_PLATE", "square")]

LIGHT_SCALE = float(os.environ.get("PAI_LIGHT_SCALE", "1.0"))
TABLE_SIZE = (0.80, 0.60, 0.04)  # x, y, thickness [m]
BLOCK_SIZE = float(os.environ.get("PAI_BLOCK_SIZE", "0.047"))  # the Isaac block is a 4.7 cm cube; scaled to this
BLOCK_MASS = float(os.environ.get("PAI_BLOCK_MASS", "0.02"))
BLOCK_COLOR = os.environ.get("PAI_BLOCK_COLOR", "red")
# The real block is an unpainted 30 mm beech cube (bought 10/31); its color varies with light and grain.
WOOD_COLOR = (0.86, 0.62, 0.42)  # warmer than the first try (0.80, 0.64, 0.50), which rendered greyish
# Robot base at the origin facing +y (LeIsaac SO-101 USD convention). The real 90 x 40 cm table spans
# x -0.20..0.70 and y -0.09..0.31 around the base (assets/scene/workspace.align.json), so the plate sits front-right
# and the block front-left, both within ~20 cm of the base. Place the real ones at the same spots.
BLOCK_INIT = (-0.08, 0.17, BLOCK_SIZE / 2)
PLATE_INIT = (0.15, 0.18, 0.0)
BLOCK_RANGE = (0.05, 0.04)  # +- x, y around BLOCK_INIT on every reset
FRONT_CAMERA: tuple | None = None  # (eye, look-at) in the env frame; None keeps the LeIsaac LiftCube pose
ROBOT_COLOR: tuple | None = None  # None keeps LeIsaac's yellow 3D-printed parts
FRONT_FOCAL_MM: float | None = None
LAYOUT = os.environ.get("PAI_LAYOUT", "default")
if LAYOUT == "real1031":
    # Measured on the real setup (10/31): plate 5 cm left of and 17 cm in front of the base, block to the plate's
    # right, the C920 on the table 45 cm right / 24 cm front / 15 cm up looking left across the plate (aimed so the
    # plate lands where it does in the real image), and a white-printed arm.
    # The plate was first measured at y = 0.24, but its 9 cm radius then reached the table's front edge (0.33 m
    # measured, 0.313 m in the 3DGS); moved to 0.17 in both sim and real at the user's request (10/31).
    PLATE_INIT = (-0.05, 0.17, 0.0)
    BLOCK_INIT = (0.12, 0.17, BLOCK_SIZE / 2)
    BLOCK_RANGE = (0.03, 0.04)
    FRONT_CAMERA = ((0.45, 0.24, 0.15), (-0.047, 0.142, -0.036))  # yaw/pitch matched to the C920 image (10/31)
    FRONT_FOCAL_MM = 42.3  # ~711 px at 640 wide: the 18 cm plate spans 256 px at ~0.5 m in the C920 image
    ROBOT_COLOR = (0.92, 0.92, 0.90)
elif LAYOUT != "default":
    raise ValueError(f"PAI_LAYOUT must be default or real1031, got {LAYOUT!r}")
REST_POSE_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -100.0,
    "elbow_flex": 90.0,
    "wrist_flex": 50.0,
    "wrist_roll": 0.0,
    "gripper": -10.0,
}


def look_at_quat(eye, target) -> tuple[float, float, float, float]:
    """(w, x, y, z) of an OpenGL-convention camera (looks along -Z, +Y up) at `eye` looking at `target`."""
    import numpy as np

    from so101_pai.recon.align import _quat_wxyz

    forward = np.subtract(target, eye) / np.linalg.norm(np.subtract(target, eye))
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    return tuple(float(v) for v in _quat_wxyz(np.column_stack([right, np.cross(right, forward), -forward])))


def spawn_recolored_robot(prim_path: str, cfg: sim_utils.UsdFileCfg, *args, **kwargs) -> Usd.Prim:
    """spawn_from_usd, then repaint the SO-101's 3D-printed parts (one shared OmniPBR material) in ROBOT_COLOR."""
    from pxr import Gf

    prim = sim_utils.spawn_from_usd(prim_path, cfg, *args, **kwargs)
    for robot in sim_utils.find_matching_prims(prim_path):
        shader = robot.GetStage().GetPrimAtPath(f"{robot.GetPath()}/Looks/material_a_3d_printed/Shader")
        if not shader.IsValid():
            raise RuntimeError(f"no 3D-printed material under {robot.GetPath()}; the SO-101 USD changed")
        shader.GetAttribute("inputs:diffuse_color_constant").Set(Gf.Vec3f(*ROBOT_COLOR))
    return prim


def spawn_fixed_plate(prim_path: str, cfg: sim_utils.UsdFileCfg, *args, **kwargs) -> Usd.Prim:
    """spawn_from_usd that really applies cfg.rigid_props.

    The SimReady plate keeps its rigid body on an instanceable child prim, which Isaac Lab cannot edit (it warns
    "Could not perform 'modify_rigid_body_properties'" and leaves the plate dynamic), so un-instance it first.
    """
    prim = sim_utils.spawn_from_usd(prim_path, cfg.replace(rigid_props=None), *args, **kwargs)
    for plate in sim_utils.find_matching_prims(prim_path):
        for child in [p for p in Usd.PrimRange(plate) if p.IsInstance()]:
            child.SetInstanceable(False)
        sim_utils.modify_rigid_body_properties(str(plate.GetPath()), cfg.rigid_props)
    return prim


@configclass
class PlaceBlockSceneCfg(SingleArmTaskSceneCfg):
    scene: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=TABLE_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.45, 0.35), roughness=0.8),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.15, -TABLE_SIZE[2] / 2)),
    )

    block: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block",
        spawn=sim_utils.UsdFileCfg(
            usd_path=BLOCK_USD,
            scale=(BLOCK_SIZE / 0.047,) * 3,
            visual_material=(
                sim_utils.PreviewSurfaceCfg(diffuse_color=WOOD_COLOR, roughness=0.7) if BLOCK_COLOR == "wood" else None
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=BLOCK_MASS),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=BLOCK_INIT),
    )

    plate: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Plate",
        spawn=sim_utils.UsdFileCfg(
            func=spawn_fixed_plate,
            usd_path=PLATE_USD,
            scale=PLATE_SCALE,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.92, 0.92, 0.9), roughness=0.85),
            # The real paper plate is taped to the table, so the sim plate does not move.
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=PLATE_INIT),
    )

    front: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/front_camera",
        # Pose from LeIsaac LiftCube, kept for the real setup (decided 10/31): the training data uses it, and the 3DGS
        # background is only sharp from this side. The real camera is placed to match it (robot/camera_overlay.py).
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.6, -0.75, 0.38), rot=(0.77337, 0.55078, -0.2374, -0.20537), convention="opengl"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=40.6,
            focus_distance=400.0,
            horizontal_aperture=38.11,
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,
    )

    # Lights only shade the sim objects (robot, block, plate); the 3DGS background carries its own lighting.
    # PAI_LIGHT_SCALE tunes both so the sim objects match the background's brightness.
    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=1000.0 * LIGHT_SCALE),
    )
    ceiling = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/CeilingLight",
        spawn=sim_utils.DistantLightCfg(color=(1.0, 0.97, 0.92), intensity=1500.0 * LIGHT_SCALE, angle=30.0),
    )


@configclass
class TerminationsCfg(SingleArmTerminationsCfg):
    success = DoneTerm(
        func=mdp.block_on_plate_released,
        params={
            "block_cfg": SceneEntityCfg("block"),
            "plate_cfg": SceneEntityCfg("plate"),
            "ee_frame_cfg": SceneEntityCfg("ee_frame"),
        },
    )


@configclass
class PlaceBlockEnvCfg(SingleArmTaskEnvCfg):
    scene: PlaceBlockSceneCfg = PlaceBlockSceneCfg(env_spacing=4.0)
    observations: SingleArmObservationsCfg = SingleArmObservationsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Episodes are recorded through ROS 2 rosbags. The template's HDF5 recorder would also dump every episode,
    # camera frames included, to /tmp/isaaclab/logs on each reset, which stalled the sim for 5-6 s.
    recorders: RecorderManagerBaseCfg = RecorderManagerBaseCfg()

    task_description: str = "Put the red block on the plate."
    block_size: float = BLOCK_SIZE  # the scripted expert plans its grasp with it

    def use_teleop_device(self, teleop_device) -> None:
        super().use_teleop_device(teleop_device)
        # Leader/policy commands are absolute joint angles (contract radians). Isaac Lab adds the default
        # joint pose by default, which is non-zero here because the arm starts at the rest pose.
        for term in ("arm_action", "gripper_action"):
            action = getattr(self.actions, term, None)
            if action is not None and hasattr(action, "use_default_offset"):
                action.use_default_offset = False

    def __post_init__(self) -> None:
        super().__post_init__()
        # 45 s (was 30): policy successes in the sim came at 21-32 s when replanning every 15-25 actions.
        self.episode_length_s = 45.0
        # Physics at 60 Hz, control + camera rendering at 30 Hz (= dataset fps). Rendering two cameras every
        # 60 Hz step only reached ~17 Hz wall-clock on the RTX 5060 Ti.
        self.sim.dt = 1 / 60
        self.decimation = 2
        self.sim.render_interval = self.decimation
        # GUI view from just behind and above the robot, so left/right on screen match the operator's leader arm.
        self.viewer.eye = (0.05, -0.30, 0.55)
        self.viewer.lookat = (0.05, 0.15, 0.0)
        self.scene.robot.init_state.pos = (0.0, 0.0, 0.0)
        # Start folded like the real arm at rest (LeIsaac SO101_FOLLOWER_REST_POSE_RANGE centers, degrees).
        self.scene.robot.init_state.joint_pos = {name: math.radians(deg) for name, deg in REST_POSE_DEG.items()}
        if ROBOT_COLOR is not None:
            self.scene.robot.spawn.func = spawn_recolored_robot
        if FRONT_CAMERA is not None:
            eye, target = FRONT_CAMERA
            self.scene.front.prim_path = "{ENV_REGEX_NS}/FrontCamera"  # in the env frame, not the robot base's
            self.scene.front.offset = TiledCameraCfg.OffsetCfg(
                pos=eye, rot=look_at_quat(eye, target), convention="opengl"
            )
        if FRONT_FOCAL_MM is not None:
            self.scene.front.spawn.focal_length = FRONT_FOCAL_MM

        add_background(self.scene)

        domain_randomization(
            self,
            random_options=[
                randomize_object_uniform(
                    "block",
                    pose_range={
                        "x": (-BLOCK_RANGE[0], BLOCK_RANGE[0]),
                        "y": (-BLOCK_RANGE[1], BLOCK_RANGE[1]),
                        "z": (0.0, 0.0),
                        "yaw": (-45 * torch.pi / 180, 45 * torch.pi / 180),
                    },
                ),
                randomize_camera_uniform(
                    "front",
                    pose_range={
                        "x": (-0.01, 0.01),
                        "y": (-0.01, 0.01),
                        "z": (-0.01, 0.01),
                        "roll": (-1 * torch.pi / 180, 1 * torch.pi / 180),
                        "pitch": (-1 * torch.pi / 180, 1 * torch.pi / 180),
                        "yaw": (-1 * torch.pi / 180, 1 * torch.pi / 180),
                    },
                    convention="opengl",
                ),
            ],
        )

        if BLOCK_COLOR == "wood":
            self.events.block_color = EventTerm(
                func=mdp.randomize_block_color,
                mode="reset",
                params={
                    "asset_cfg": SceneEntityCfg("block"),
                    "base": WOOD_COLOR,
                    "brightness": (0.8, 1.15),
                    "jitter": 0.04,
                },
            )


def add_background(scene: PlaceBlockSceneCfg) -> None:
    """Add the 3DGS background (NuRec USDZ from scripts/recon/run.sh) when PAI_BACKGROUND_USD is set.

    The reconstruction is in COLMAP's frame, so its pose comes from a sidecar JSON next to the USDZ
    (<name>.align.json from `so101_pai.recon.align`: {"pos": [x, y, z], "rot": [w, x, y, z], "scale": s});
    identity if it does not exist. The real table is then part of the background, so the sim table box is hidden
    and only its collision remains, resized and moved to the measured table ("table" in the JSON) when present.
    """
    usd = os.environ.get("PAI_BACKGROUND_USD")
    if not usd:
        return
    align_path = Path(usd).with_suffix(".align.json")
    align = json.loads(align_path.read_text()) if align_path.exists() else {}
    scale = float(align.get("scale", 1.0))
    scene.background = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Background",
        spawn=sim_utils.UsdFileCfg(usd_path=str(Path(usd).resolve()), scale=(scale, scale, scale)),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(align.get("pos", (0.0, 0.0, 0.0))), rot=tuple(align.get("rot", (1.0, 0.0, 0.0, 0.0)))
        ),
    )
    scene.scene.spawn.visible = False
    if "table" in align:
        (x, y), (sx, sy) = align["table"]["center"], align["table"]["size"]
        scene.scene.spawn.size = (sx, sy, TABLE_SIZE[2])
        scene.scene.init_state.pos = (x, y, -TABLE_SIZE[2] / 2)
