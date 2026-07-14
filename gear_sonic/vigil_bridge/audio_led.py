"""Audio-reactive G1 speaker LED planning.

The bridge computes a compact RGB plan from the exact outgoing PCM.  The
native speaker runner applies the plan while the same PCM is playing, keeping
audio timing and LED timing in one process.  During speech every generated
frame has a zero blue channel; blue is introduced only by the runner after
audio playback has stopped.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Any


JSONDict = dict[str, Any]


@dataclass(frozen=True)
class ReactiveLedConfig:
    refresh_hz: int = 50
    rms_reference_quantile: float = 0.99
    rms_reference_headroom: float = 1.08
    noise_floor_ratio: float = 0.035
    noise_floor_min: float = 180.0
    amplitude_exponent: float = 1.8
    attack_alpha: float = 0.18
    release_alpha: float = 0.045
    display_alpha: float = 0.18
    breath_hz: float = 0.3
    breath_depth: float = 0.1
    speech_floor: float = 0.12
    visual_reference_quantile: float = 0.99
    end_to_dark_blue_ms: int = 250
    dark_to_bright_blue_ms: int = 500
    bright_blue_hold_ms: int = 500
    dark_blue_rgb: tuple[int, int, int] = (0, 0, 40)
    bright_blue_rgb: tuple[int, int, int] = (0, 0, 255)


DEFAULT_REACTIVE_LED_CONFIG = ReactiveLedConfig()


def build_reactive_led_plan(
    pcm: bytes,
    *,
    sample_rate: int = 16000,
    channels: int = 1,
    sample_width: int = 2,
    config: ReactiveLedConfig = DEFAULT_REACTIVE_LED_CONFIG,
) -> tuple[bytes, JSONDict]:
    """Return packed RGB frames and runtime telemetry for normalized PCM16."""

    if sample_rate <= 0 or sample_rate % config.refresh_hz:
        raise ValueError("sample_rate must be a positive multiple of LED refresh_hz")
    if channels != 1 or sample_width != 2:
        raise ValueError("reactive LED planning requires mono PCM16")
    if len(pcm) % sample_width:
        pcm = pcm[: -(len(pcm) % sample_width)]
    if not pcm:
        raise ValueError("reactive LED planning requires non-empty PCM")

    samples = struct.unpack("<" + "h" * (len(pcm) // 2), pcm)
    frame_samples = sample_rate // config.refresh_hz
    rms_values = [
        _rms(samples[start : start + frame_samples])
        for start in range(0, len(samples), frame_samples)
    ]
    rms_reference = max(
        _quantile(rms_values, config.rms_reference_quantile) * config.rms_reference_headroom,
        1.0,
    )
    noise_floor = max(config.noise_floor_min, rms_reference * config.noise_floor_ratio)

    visual_levels: list[float] = []
    envelope = 0.0
    display = 0.0
    for index, rms in enumerate(rms_values):
        linear = _clamp((rms - noise_floor) / max(rms_reference - noise_floor, 1.0))
        target = linear**config.amplitude_exponent
        alpha = config.attack_alpha if target > envelope else config.release_alpha
        envelope += alpha * (target - envelope)
        display += config.display_alpha * (envelope - display)
        breath = 0.5 - 0.5 * math.cos(
            2.0 * math.pi * config.breath_hz * index / config.refresh_hz
        )
        visual_levels.append(
            max(
                config.speech_floor,
                display * (1.0 - config.breath_depth + config.breath_depth * breath),
            )
        )

    visual_reference = max(
        _quantile(visual_levels, config.visual_reference_quantile),
        config.speech_floor + 0.01,
    )
    colors = [
        speech_color(
            _clamp(
                (level - config.speech_floor)
                / max(visual_reference - config.speech_floor, 0.01)
            )
        )
        for level in visual_levels
    ]
    if any(blue != 0 for _, _, blue in colors):
        raise AssertionError("speech LED plan must not contain blue")

    packed = bytes(channel for color in colors for channel in color)
    return packed, {
        "enabled": True,
        "refresh_hz": config.refresh_hz,
        "frame_count": len(colors),
        "duration_s": len(samples) / float(sample_rate),
        "rms_reference": rms_reference,
        "noise_floor": noise_floor,
        "visual_reference": visual_reference,
        "speech_rgb_low": list(speech_color(0.0)),
        "speech_rgb_mid": list(speech_color(0.5)),
        "speech_rgb_high": list(speech_color(1.0)),
        "speech_blue_channel": 0,
        "end_animation": {
            "to_dark_blue_ms": config.end_to_dark_blue_ms,
            "dark_to_bright_blue_ms": config.dark_to_bright_blue_ms,
            "bright_blue_hold_ms": config.bright_blue_hold_ms,
            "dark_blue_rgb": list(config.dark_blue_rgb),
            "bright_blue_rgb": list(config.bright_blue_rgb),
        },
    }


def speech_color(contrast: float) -> tuple[int, int, int]:
    """Map contrast to saturated dark-orange, amber, and bright-yellow RGB."""

    contrast = _clamp(contrast)
    if contrast < 0.5:
        amount = _smoothstep(contrast / 0.5)
        hue = _lerp(18.0, 30.0, amount)
        value = _lerp(0.18, 0.62, amount)
    else:
        amount = _smoothstep((contrast - 0.5) / 0.5)
        hue = _lerp(30.0, 55.0, amount)
        value = _lerp(0.62, 1.0, amount)
    return _hsv_full_saturation(hue, value)


def reactive_led_capabilities(enabled: bool) -> JSONDict:
    config = DEFAULT_REACTIVE_LED_CONFIG
    return {
        "enabled": enabled,
        "source": "outgoing_pcm_amplitude",
        "refresh_hz": config.refresh_hz,
        "speech_palette": {
            "low": list(speech_color(0.0)),
            "mid": list(speech_color(0.5)),
            "high": list(speech_color(1.0)),
            "blue_channel": 0,
        },
        "end_animation": {
            "to_dark_blue_ms": config.end_to_dark_blue_ms,
            "dark_to_bright_blue_ms": config.dark_to_bright_blue_ms,
            "bright_blue_hold_ms": config.bright_blue_hold_ms,
            "dark_blue_rgb": list(config.dark_blue_rgb),
            "bright_blue_rgb": list(config.bright_blue_rgb),
        },
        "native_tts_amplitude_available": False,
    }


def _rms(samples: tuple[int, ...]) -> float:
    if not samples:
        return 0.0
    return math.sqrt(sum(float(value) * value for value in samples) / len(samples))


def _quantile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(_clamp(quantile) * (len(ordered) - 1))
    return ordered[index]


def _hsv_full_saturation(hue_degrees: float, value: float) -> tuple[int, int, int]:
    hue = max(hue_degrees, 0.0) % 360.0
    chroma = _clamp(value)
    sector = hue / 60.0
    secondary = chroma * (1.0 - abs((sector % 2.0) - 1.0))
    if sector < 1.0:
        red, green = chroma, secondary
    elif sector < 2.0:
        red, green = secondary, chroma
    else:
        red, green = 0.0, 0.0
    return round(255.0 * red), round(255.0 * green), 0


def _smoothstep(value: float) -> float:
    value = _clamp(value)
    return value * value * (3.0 - 2.0 * value)


def _lerp(start: float, end: float, amount: float) -> float:
    return start + (end - start) * amount


def _clamp(value: float) -> float:
    return max(0.0, min(float(value), 1.0))
