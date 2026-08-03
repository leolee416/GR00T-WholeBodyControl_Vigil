"""Entry point for running the repository's G1 MuJoCo simulation loop.

Parses a YAML-based WBC config via tyro CLI and launches the simulator,
optionally with offscreen image publishing.
"""

from typing import Any, Dict

import tyro

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory

ArgsConfig = SimLoopConfig


class SimWrapper:
    def __init__(
        self,
        robot_model: Any,
        env_name: str,
        config: Dict[str, Any],
        **kwargs: Any,
    ) -> None:
        self.robot_model = robot_model
        self.config = config

        # Create simulator using factory
        self.sim = SimulatorFactory.create_simulator(
            config=self.config,
            env_name=env_name,
            **kwargs,
        )


def main(config: ArgsConfig):
    wbc_config = config.load_wbc_yaml()
    # NOTE: we will override the interface to local if it is not specified
    wbc_config["ENV_NAME"] = config.env_name

    if config.enable_image_publish:
        assert (
            config.enable_offscreen
        ), "enable_offscreen must be True when enable_image_publish is True"

    sim_wrapper = SimWrapper(
        # The legacy MuJoCo simulator constructs its runtime Robot directly
        # from wbc_config; SimWrapper only retains this field for API
        # compatibility.  Avoid importing Pinocchio/URDF tooling in the
        # standalone simulation process when it is not used.
        robot_model=None,
        env_name=config.env_name,
        config=wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=config.enable_image_publish,
    )
    # Start simulator as independent process
    SimulatorFactory.start_simulator(
        sim_wrapper.sim,
        as_thread=False,
        enable_image_publish=config.enable_image_publish,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )


if __name__ == "__main__":
    config = tyro.cli(ArgsConfig)
    main(config)
