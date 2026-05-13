"""HTTP transport for the GR00T-side Vigil bridge service."""

from __future__ import annotations

import json
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable

from gear_sonic.vigil_bridge.protocol import BRIDGE_NAME, BRIDGE_VERSION, PROTOCOL_VERSION
from gear_sonic.vigil_bridge.service import VigilBridgeService

JSONResponse = dict[str, Any]


@dataclass(frozen=True)
class HTTPTransportConfig:
    host: str = "127.0.0.1"
    port: int = 8765
    runtime_mode: str = "dry_run"


class BridgeRequestRouter:
    """Maps HTTP endpoint names to `VigilBridgeService` calls."""

    def __init__(self, service: VigilBridgeService) -> None:
        self.service = service

    def dispatch(self, endpoint: str, payload: dict[str, Any] | None = None) -> JSONResponse:
        request_payload = payload or {}
        route = endpoint.strip("/")

        if route == "handshake":
            return self.service.handshake(request_payload)
        if route == "reset_episode":
            return self.service.reset_episode(request_payload)
        if route == "execute_action":
            return self.service.execute_action(request_payload)
        if route in {"observation", "get_observation"}:
            return self.service.get_observation(request_payload)
        if route in {"robot_state", "get_robot_state"}:
            return self.service.get_robot_state()
        if route == "halt":
            return self.service.halt()
        if route == "close":
            self.service.close()
            return {
                "ok": True,
                "error_message": None,
                "bridge": {
                    "name": BRIDGE_NAME,
                    "version": BRIDGE_VERSION,
                },
            }
        if route == "health":
            return {
                "ok": True,
                "error_message": None,
                "protocol_version": PROTOCOL_VERSION,
                "runtime_mode": self.service.runtime_mode,
                "bridge": {
                    "name": BRIDGE_NAME,
                    "version": BRIDGE_VERSION,
                },
            }

        return {
            "ok": False,
            "error_message": f"unsupported endpoint: {endpoint}",
        }


def create_http_server(
    host: str = "127.0.0.1",
    port: int = 8765,
    service: VigilBridgeService | None = None,
) -> ThreadingHTTPServer:
    bridge_service = service or VigilBridgeService()
    router = BridgeRequestRouter(bridge_service)

    class VigilBridgeHTTPHandler(BaseHTTPRequestHandler):
        server_version = "GrootVigilBridgeHTTP/0.1"

        def do_GET(self) -> None:
            if self.path.rstrip("/") == "/health":
                self._write_json(HTTPStatus.OK, router.dispatch("health", {}))
                return
            self._write_json(
                HTTPStatus.NOT_FOUND,
                {
                    "ok": False,
                    "error_message": f"unsupported endpoint: {self.path}",
                },
            )

        def do_POST(self) -> None:
            payload = self._read_json_body()
            if payload is None:
                return

            endpoint = self.path.split("?", 1)[0].strip("/")
            try:
                response = router.dispatch(endpoint, payload)
            except Exception as exc:
                response = {
                    "ok": False,
                    "error_message": str(exc),
                }
            status = HTTPStatus.OK if endpoint in _known_routes() else HTTPStatus.NOT_FOUND
            self._write_json(status, response)

        def log_message(self, format: str, *args: Any) -> None:
            return

        def _read_json_body(self) -> dict[str, Any] | None:
            content_length = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(content_length) if content_length else b"{}"
            try:
                payload = json.loads(raw_body.decode("utf-8") or "{}")
            except json.JSONDecodeError as exc:
                self._write_json(
                    HTTPStatus.BAD_REQUEST,
                    {
                        "ok": False,
                        "error_message": f"invalid JSON payload: {exc.msg}",
                    },
                )
                return None
            if not isinstance(payload, dict):
                self._write_json(
                    HTTPStatus.BAD_REQUEST,
                    {
                        "ok": False,
                        "error_message": "JSON payload must be an object",
                    },
                )
                return None
            return payload

        def _write_json(self, status: HTTPStatus, response: JSONResponse) -> None:
            body = json.dumps(response, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return ThreadingHTTPServer((host, port), VigilBridgeHTTPHandler)


def serve_http(
    host: str = "127.0.0.1",
    port: int = 8765,
    runtime_mode: str = "dry_run",
    service_factory: Callable[[], VigilBridgeService] | None = None,
) -> None:
    service = service_factory() if service_factory is not None else VigilBridgeService(runtime_mode=runtime_mode)
    server = create_http_server(host=host, port=port, service=service)
    try:
        server.serve_forever()
    finally:
        service.close()
        server.server_close()


def _known_routes() -> set[str]:
    return {
        "handshake",
        "reset_episode",
        "execute_action",
        "observation",
        "get_observation",
        "robot_state",
        "get_robot_state",
        "halt",
        "close",
        "health",
    }
