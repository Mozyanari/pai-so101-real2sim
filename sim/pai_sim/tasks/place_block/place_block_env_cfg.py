"""PlaceBlock: pick a 4.7 cm block and place it on a square plate with the SO-101.

Built on LeIsaac's single-arm template (Apache-2.0). Assets are NVIDIA-official:
Isaac Sim 5.1 Props/Blocks/red_block.usd and SimReady dishware_plastic_smallPlate_a01
(both fetched by setup/04_task_assets.sh).
The plate is rescaled and recolored white to match the real ~18 cm round paper plate.
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
# The asset is a round 9.9 cm x 4.5 cm deep plate; flatten and widen it to a ~17.8 cm x 1.8 cm paper plate.
PLATE_SCALE = (1.8, 1.8, 0.4)

LIGHT_SCALE = float(os.environ.get("PAI_LIGHT_SCALE", "1.0"))
TABLE_SIZE = (0.80, 0.60, 0.04)  # x, y, thickness [m]
BLOCK_SIZE = 0.047
# Robot base at the origin facing +y (LeIsaac SO-101 USD convention). The real 90 x 40 cm table spans
# x -0.20..0.70 and y -0.09..0.31 around the base (assets/scene/workspace.align.json), so the plate sits front-right
# and the block front-left, both within ~20 cm of the base. Place the real ones at the same spots.
BLOCK_INIT = (-0.08, 0.17, BLOCK_SIZE / 2)
PLATE_INIT = (0.15, 0.18, 0.0)
REST_POSE_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -100.0,
    "elbow_flex": 90.0,
    "wrist_flex": 50.0,
    "wrist_roll": 0.0,
    "gripper": -10.0,
}


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
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
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
        # Placeholder pose from LeIsaac LiftCube; replaced by the measured real camera pose.
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
        self.episode_length_s = 30.0
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

        add_background(self.scene)

        domain_randomization(
            self,
            random_options=[
                randomize_object_uniform(
                    "block",
                    pose_range={
                        "x": (-0.05, 0.05),
                        "y": (-0.04, 0.04),
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
