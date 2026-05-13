"""Fake sensor provider for the GR00T-side Vigil bridge skeleton."""

from __future__ import annotations

from dataclasses import dataclass

from gear_sonic.vigil_bridge.protocol import ObservationResponse, RobotStateResponse


@dataclass
class FakeSensorProvider:
    """Sensor provider that returns JSON-compatible fake payloads only."""

    runtime_mode: str = "dry_run"
    connected: bool = True
    _observation_index: int = 0
    _state_index: int = 0

    def get_observation(self) -> ObservationResponse:
        self._observation_index += 1
        robot_state = self._make_robot_state(state_index=self._state_index)
        return {
            "observation_id": f"fake_obs_{self._observation_index:04d}",
            "runtime_mode": self.runtime_mode,
            "images": {
                "ego_view": {
                    "encoding": "fake-rgb-base64",
                    "width": 1,
                    "height": 1,
                    "data": "AAAA",
                }
            },
            "camera_timestamps": {"ego_view": float(self._observation_index)},
            "robot_state": robot_state,
            "telemetry": {
                "sensor_provider": "fake",
                "hardware": False,
                "connected": self.connected,
            },
            "perception": {
                "source": "none",
                "detections": [],
            },
        }

    def get_robot_state(self) -> RobotStateResponse:
        self._state_index += 1
        return {
            "ok": self.connected,
            "error_message": None if self.connected else "fake sensor provider disconnected",
            "runtime_mode": self.runtime_mode,
            "robot_state": self._make_robot_state(state_index=self._state_index),
            "telemetry": {
                "sensor_provider": "fake",
                "hardware": False,
                "connected": self.connected,
            },
        }

    def _make_robot_state(self, state_index: int) -> dict:
        return {
            "state_id": f"fake_state_{state_index:04d}",
            "base_pose": {
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_deg": 0.0,
            },
            "base_velocity": {
                "linear_mps": 0.0,
                "angular_deg_s": 0.0,
            },
            "joint_positions": {},
            "estimated": False,
        }
