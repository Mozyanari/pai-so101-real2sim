import gymnasium as gym

gym.register(
    id="PAI-SO101-PlaceBlock-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.place_block_env_cfg:PlaceBlockEnvCfg",
    },
)
