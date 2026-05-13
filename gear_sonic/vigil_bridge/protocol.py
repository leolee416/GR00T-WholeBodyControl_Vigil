"""JSON/msgpack-compatible payload shapes for the GR00T-side Vigil bridge."""

from __future__ import annotations

from typing import TypeAlias, TypedDict, Union

JSONPrimitive: TypeAlias = Union[str, int, float, bool, None]
JSONValue: TypeAlias = Union[JSONPrimitive, list["JSONValue"], dict[str, "JSONValue"]]
JSONDict: TypeAlias = dict[str, JSONValue]

PROTOCOL_VERSION = "vigil_groot_bridge_v1"
BRIDGE_NAME = "gear_sonic_vigil_bridge"
BRIDGE_VERSION = "dry_run_phase1"
SUPPORTED_ACTIONS = [
    "navigate.backward",
    "navigate.forward",
    "navigate.turn_left",
    "navigate.turn_right",
]
SUPPORTED_OBSERVATIONS = ["rgb", "robot_state"]
ORACLE_SOURCE = "none"


class ClientInfo(TypedDict, total=False):
    name: str
    component: str


class RequiredCapabilities(TypedDict, total=False):
    actions: list[str]
    observation: list[str]
    oracle_source: str


class BridgeCapabilities(TypedDict):
    actions: list[str]
    observation: list[str]
    oracle_source: str


class BridgeInfo(TypedDict):
    name: str
    version: str


class HandshakeRequest(TypedDict, total=False):
    protocol_version: str
    client: ClientInfo
    episode_id: str
    runtime_mode: str
    required_capabilities: RequiredCapabilities


class HandshakeResponse(TypedDict):
    ok: bool
    error_message: str | None
    protocol_version: str
    runtime_mode: str
    capabilities: BridgeCapabilities
    bridge: BridgeInfo


class ResetEpisodeRequest(TypedDict, total=False):
    episode_id: str
    runtime_mode: str
    options: JSONDict


class ResetEpisodeResponse(TypedDict, total=False):
    ok: bool
    error_message: str | None
    episode_id: str | None
    runtime_mode: str
    robot_state: JSONDict
    telemetry: JSONDict


class ExecuteActionRequest(TypedDict, total=False):
    episode_id: str
    step_id: int
    runtime_mode: str
    skill_name: str
    arguments: JSONDict
    safety: JSONDict


class ExecuteActionResponse(TypedDict, total=False):
    ok: bool
    error_message: str | None
    action_status: str
    executed_arguments: JSONDict
    robot_state_before: JSONDict
    robot_state_after: JSONDict
    telemetry: JSONDict


class ObservationResponse(TypedDict, total=False):
    observation_id: str
    runtime_mode: str
    images: dict[str, JSONValue]
    camera_timestamps: JSONDict
    robot_state: JSONDict
    telemetry: JSONDict
    perception: JSONDict


class RobotStateResponse(TypedDict, total=False):
    ok: bool
    error_message: str | None
    runtime_mode: str
    robot_state: JSONDict
    telemetry: JSONDict


class RuntimeHealth(TypedDict, total=False):
    ok: bool
    runtime_mode: str
    executor_started: bool
    sensor_connected: bool
    error_message: str | None
    telemetry: JSONDict
