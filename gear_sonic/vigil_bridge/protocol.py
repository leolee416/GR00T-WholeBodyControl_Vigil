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
    "sonic.sit_chair",
]
SUPPORTED_OBSERVATIONS = ["rgb", "depth", "robot_state"]
AUDIO_CAPABILITIES: JSONDict = {
    "enabled": False,
    "input": ["pcm16_16k_mono_stream", "pcm16_16k_mono_segment"],
    "output": ["pcm16_16k_mono_stream", "pcm16_16k_mono_segment", "native_tts_text"],
    "transport": ["websocket", "http_segment_fallback"],
    "sample_rate": 16000,
    "channels": 1,
    "sample_width": 2,
    "speaker_volume": 100,
    "speaker_peak_target": 27800,
    "tts": {
        "languages": ["zh", "en"],
        "speaker_ids": {"zh": 0, "en": 1},
        "text_max_chars": 500,
        "native_loudness_calibrated": False,
        "calibrated_output_path": "/audio/output_segment",
    },
}
ORACLE_SOURCE = "none"


class ClientInfo(TypedDict, total=False):
    name: str
    component: str


class RequiredCapabilities(TypedDict, total=False):
    actions: list[str]
    observation: list[str]
    audio: JSONDict
    oracle_source: str


class BridgeCapabilities(TypedDict, total=False):
    actions: list[str]
    observation: list[str]
    audio: JSONDict
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


class AudioResponse(TypedDict, total=False):
    ok: bool
    error_message: str | None
    session_id: str
    encoding: str
    data: str
    duration_s: float
    format: JSONDict
    audio_stats: JSONDict
    telemetry: JSONDict
