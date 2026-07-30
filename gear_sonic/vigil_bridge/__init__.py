"""Dry-run GR00T-side bridge primitives for Vigil integration."""

from gear_sonic.vigil_bridge.audio import (
    AudioBridgeConfig,
    AudioSessionManager,
    FakeSpeakerClient,
)
from gear_sonic.vigil_bridge.primitive_executor import (
    DryRunPrimitiveExecutor,
    FakePrimitiveExecutor,
)
from gear_sonic.vigil_bridge.mujoco_adapter import (
    MujocoBridgeConfig,
    MujocoPrimitiveExecutor,
    MujocoSensorProvider,
    create_mujoco_bridge_service,
)
from gear_sonic.vigil_bridge.real_adapter import (
    RealBridgeConfig,
    RealPrimitiveExecutor,
    RealSensorProvider,
    create_real_bridge_service,
)
from gear_sonic.vigil_bridge.rollout_recorder import G1_JOINT_ORDER, RolloutRecorder
from gear_sonic.vigil_bridge.sensors import FakeSensorProvider
from gear_sonic.vigil_bridge.service import VigilBridgeService
from gear_sonic.vigil_bridge.transport import BridgeRequestRouter, create_http_server, serve_http

__all__ = [
    "AudioBridgeConfig",
    "AudioSessionManager",
    "BridgeRequestRouter",
    "DryRunPrimitiveExecutor",
    "FakePrimitiveExecutor",
    "FakeSpeakerClient",
    "FakeSensorProvider",
    "G1_JOINT_ORDER",
    "MujocoBridgeConfig",
    "MujocoPrimitiveExecutor",
    "MujocoSensorProvider",
    "RealBridgeConfig",
    "RealPrimitiveExecutor",
    "RealSensorProvider",
    "RolloutRecorder",
    "VigilBridgeService",
    "create_http_server",
    "create_mujoco_bridge_service",
    "create_real_bridge_service",
    "serve_http",
]
