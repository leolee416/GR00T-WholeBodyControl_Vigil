"""Dry-run GR00T-side bridge primitives for Vigil integration."""

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
from gear_sonic.vigil_bridge.sensors import FakeSensorProvider
from gear_sonic.vigil_bridge.service import VigilBridgeService
from gear_sonic.vigil_bridge.transport import BridgeRequestRouter, create_http_server, serve_http

__all__ = [
    "BridgeRequestRouter",
    "DryRunPrimitiveExecutor",
    "FakePrimitiveExecutor",
    "FakeSensorProvider",
    "MujocoBridgeConfig",
    "MujocoPrimitiveExecutor",
    "MujocoSensorProvider",
    "RealBridgeConfig",
    "RealPrimitiveExecutor",
    "RealSensorProvider",
    "VigilBridgeService",
    "create_http_server",
    "create_mujoco_bridge_service",
    "create_real_bridge_service",
    "serve_http",
]
