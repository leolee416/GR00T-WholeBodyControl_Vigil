#!/usr/bin/env python3
"""Fit a simple calibrated move model from Vigil WBC grid-search results.

The model is intentionally small:
  direction + abs(magnitude) -> speed, execute_time

It uses piecewise-linear interpolation over the best grid-search row for each
target magnitude. This keeps the model faithful to the calibration data while
still allowing arbitrary magnitudes between measured targets.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class SummaryRow:
    expected_magnitude: float
    best_rate: float
    best_execute_time: float
    mean_actual_magnitude: float
    mean_abs_error: float
    trials: int


@dataclass(frozen=True)
class ModelSample:
    magnitude_abs: float
    rate: float
    execute_time: float
    command_product: float
    mean_actual_abs: float
    mean_abs_error: float


def resolve_path(path_text: str | Path) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def latest_summary_file() -> Path:
    candidates = sorted((REPO_ROOT / "outputs" / "vigil_grid_plans").glob("*_summary.csv"))
    if not candidates:
        raise FileNotFoundError(
            "No grid summary CSV found under outputs/vigil_grid_plans. "
            "Pass --source PATH to a summary CSV/XML or completed plan XML."
        )
    return candidates[-1]


def read_rows(source: Path) -> list[SummaryRow]:
    if source.suffix.lower() == ".csv":
        return read_summary_csv(source)

    root = ET.parse(source).getroot()
    if root.tag == "vigil_wbc_grid_summary":
        return read_summary_xml(source)
    if root.tag == "vigil_wbc_grid_plan":
        return rows_from_plan_xml(source)
    raise ValueError(f"Unsupported XML root tag: {root.tag}")


def read_summary_csv(path: Path) -> list[SummaryRow]:
    rows: list[SummaryRow] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                SummaryRow(
                    expected_magnitude=float(row["expected_magnitude"]),
                    best_rate=float(row["best_rate"]),
                    best_execute_time=float(row["best_excute_time"]),
                    mean_actual_magnitude=float(row["mean_actual_magnitude"]),
                    mean_abs_error=float(row["mean_abs_error"]),
                    trials=int(float(row.get("trials", 0) or 0)),
                )
            )
    return rows


def read_summary_xml(path: Path) -> list[SummaryRow]:
    root = ET.parse(path).getroot()
    rows: list[SummaryRow] = []
    for target in root.findall("./target"):
        rows.append(
            SummaryRow(
                expected_magnitude=float(target.get("expected_magnitude", "0")),
                best_rate=float(target.get("best_rate", "0")),
                best_execute_time=float(target.get("best_excute_time", "0")),
                mean_actual_magnitude=float(target.get("mean_actual_magnitude", "0")),
                mean_abs_error=float(target.get("mean_abs_error", "0")),
                trials=int(float(target.get("trials", "0"))),
            )
        )
    return rows


def rows_from_plan_xml(path: Path) -> list[SummaryRow]:
    """Build the same best-per-target summary directly from a grid plan XML."""
    root = ET.parse(path).getroot()
    grouped: dict[tuple[float, float, float], list[dict[str, float]]] = {}
    for run in root.findall("./runs/run"):
        if run.get("status") != "completed":
            continue
        actual = run.get("actual_magnitude")
        abs_error = run.get("abs_error")
        if actual is None or abs_error is None:
            continue
        expected = float(run.get("expected_magnitude", "0"))
        rate = float(run.get("rate", "0"))
        execute_time = float(run.get("excute_time", "0"))
        grouped.setdefault((expected, rate, execute_time), []).append(
            {
                "actual": float(actual),
                "abs_error": float(abs_error),
            }
        )

    best_by_target: dict[float, SummaryRow] = {}
    for (expected, rate, execute_time), trials in grouped.items():
        mean_actual = sum(row["actual"] for row in trials) / len(trials)
        mean_abs_error = sum(row["abs_error"] for row in trials) / len(trials)
        current = best_by_target.get(expected)
        if current is None or mean_abs_error < current.mean_abs_error:
            best_by_target[expected] = SummaryRow(
                expected_magnitude=expected,
                best_rate=rate,
                best_execute_time=execute_time,
                mean_actual_magnitude=mean_actual,
                mean_abs_error=mean_abs_error,
                trials=len(trials),
            )
    return [best_by_target[key] for key in sorted(best_by_target)]


def build_samples(rows: Iterable[SummaryRow], sign: int) -> list[ModelSample]:
    selected = [
        row for row in rows
        if (row.expected_magnitude > 0.0 and sign > 0) or (row.expected_magnitude < 0.0 and sign < 0)
    ]
    samples = [
        ModelSample(
            magnitude_abs=abs(row.expected_magnitude),
            rate=abs(row.best_rate),
            execute_time=row.best_execute_time,
            command_product=abs(row.best_rate * row.best_execute_time),
            mean_actual_abs=abs(row.mean_actual_magnitude),
            mean_abs_error=row.mean_abs_error,
        )
        for row in selected
    ]
    return sorted(samples, key=lambda row: row.magnitude_abs)


def interp_linear(x: float, points: list[tuple[float, float]]) -> float:
    if not points:
        raise ValueError("Cannot interpolate with no points.")
    if len(points) == 1:
        return points[0][1]

    if x <= points[0][0]:
        return extrapolate(x, points[0], points[1])
    if x >= points[-1][0]:
        return extrapolate(x, points[-2], points[-1])

    for left, right in zip(points, points[1:]):
        if left[0] <= x <= right[0]:
            return extrapolate(x, left, right)
    return points[-1][1]


def extrapolate(x: float, left: tuple[float, float], right: tuple[float, float]) -> float:
    dx = right[0] - left[0]
    if abs(dx) < 1e-9:
        return left[1]
    alpha = (x - left[0]) / dx
    return left[1] + alpha * (right[1] - left[1])


def predict(samples: list[ModelSample], magnitude_abs: float) -> tuple[float, float, float]:
    rate = interp_linear(magnitude_abs, [(sample.magnitude_abs, sample.rate) for sample in samples])
    execute_time = interp_linear(
        magnitude_abs,
        [(sample.magnitude_abs, sample.execute_time) for sample in samples],
    )
    return rate, execute_time, rate * execute_time


def predict_signed(
    forward: list[ModelSample],
    backward: list[ModelSample],
    magnitude: float,
) -> tuple[float, float, float]:
    if abs(magnitude) < 1e-9:
        return 0.0, 0.0, 0.0
    sign = 1.0 if magnitude > 0.0 else -1.0
    samples = forward if sign > 0.0 else backward
    rate, execute_time, command_product = predict(samples, abs(magnitude))
    return sign * rate, execute_time, sign * command_product


def dense_targets(samples: list[ModelSample], step: float) -> list[float]:
    if not samples:
        return []
    start = samples[0].magnitude_abs
    end = samples[-1].magnitude_abs
    count = max(1, int(math.floor((end - start) / step)))
    targets = [round(start + idx * step, 6) for idx in range(count + 1)]
    if targets[-1] < end - 1e-9:
        targets.append(end)
    return targets


def write_model_json(
    path: Path,
    source: Path,
    forward: list[ModelSample],
    backward: list[ModelSample],
) -> None:
    payload = {
        "generated_at": time.strftime("%y%m%d_%H%M%S"),
        "source": str(source),
        "model_type": "piecewise_linear_abs_magnitude_to_rate_and_execute_time",
        "usage": {
            "input": "signed target magnitude in meters",
            "forward": "use when magnitude > 0",
            "backward": "use when magnitude < 0",
            "prediction": "interpolate abs(magnitude) -> rate, execute_time; command signed move(magnitude, rate, execute_time)",
        },
        "models": {
            "forward": samples_to_json(forward),
            "backward": samples_to_json(backward),
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def samples_to_json(samples: list[ModelSample]) -> list[dict[str, float]]:
    return [
        {
            "magnitude_abs": sample.magnitude_abs,
            "rate": sample.rate,
            "execute_time": sample.execute_time,
            "command_product": sample.command_product,
            "mean_actual_abs": sample.mean_actual_abs,
            "mean_abs_error": sample.mean_abs_error,
        }
        for sample in samples
    ]


def write_prediction_table(
    path: Path,
    forward: list[ModelSample],
    backward: list[ModelSample],
    step: float,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "direction",
            "target_magnitude_abs",
            "v_model",
            "t_model",
            "magnitude_model_vt",
            "default_expected_magnitude_vt",
            "model_minus_default",
        ])
        for name, samples in (("forward", forward), ("backward", backward)):
            for target in dense_targets(samples, step):
                rate, execute_time, command_product = predict(samples, target)
                writer.writerow([
                    name,
                    f"{target:.6f}",
                    f"{rate:.6f}",
                    f"{execute_time:.6f}",
                    f"{command_product:.6f}",
                    f"{target:.6f}",
                    f"{command_product - target:.6f}",
                ])


def signed_targets(min_magnitude: float, max_magnitude: float, step: float) -> list[float]:
    if step <= 0.0:
        raise ValueError("--plot-step must be positive.")
    if min_magnitude > max_magnitude:
        min_magnitude, max_magnitude = max_magnitude, min_magnitude
    count = int(math.floor((max_magnitude - min_magnitude) / step))
    targets = [round(min_magnitude + idx * step, 6) for idx in range(count + 1)]
    if not targets or targets[-1] < max_magnitude - 1e-9:
        targets.append(round(max_magnitude, 6))
    return targets


def write_signed_vt_table(
    path: Path,
    forward: list[ModelSample],
    backward: list[ModelSample],
    min_magnitude: float,
    max_magnitude: float,
    step: float,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "target_magnitude",
            "direction",
            "signed_v_model",
            "speed_abs",
            "t_model",
            "signed_magnitude_model_vt",
            "default_expected_magnitude",
            "model_minus_default",
            "is_extrapolated",
        ])
        for target in signed_targets(min_magnitude, max_magnitude, step):
            signed_rate, execute_time, signed_product = predict_signed(forward, backward, target)
            samples = forward if target >= 0.0 else backward
            max_trained = samples[-1].magnitude_abs
            min_trained = samples[0].magnitude_abs
            is_extrapolated = abs(target) > max_trained + 1e-9 or (abs(target) < min_trained - 1e-9 and abs(target) > 1e-9)
            writer.writerow([
                f"{target:.6f}",
                "forward" if target >= 0.0 else "backward",
                f"{signed_rate:.6f}",
                f"{abs(signed_rate):.6f}",
                f"{execute_time:.6f}",
                f"{signed_product:.6f}",
                f"{target:.6f}",
                f"{signed_product - target:.6f}",
                str(is_extrapolated).lower(),
            ])


def write_svg_plot(
    path: Path,
    forward: list[ModelSample],
    backward: list[ModelSample],
    step: float,
) -> None:
    width = 1100
    height = 520
    margin = 62
    gap = 80
    panel_w = (width - 2 * margin - gap) / 2
    panel_h = height - 2 * margin

    all_targets = dense_targets(forward, step) + dense_targets(backward, step)
    max_x = max(all_targets) if all_targets else 1.0
    all_products: list[float] = []
    for samples in (forward, backward):
        for target in dense_targets(samples, step):
            all_products.append(predict(samples, target)[2])
        all_products.extend(sample.command_product for sample in samples)
    max_y = max([max_x, *all_products]) if all_products else max_x
    max_y *= 1.08

    def sx(panel_index: int, x: float) -> float:
        left = margin + panel_index * (panel_w + gap)
        return left + (x / max_x) * panel_w

    def sy(y: float) -> float:
        return margin + panel_h - (y / max_y) * panel_h

    def path_data(panel_index: int, samples: list[ModelSample], model: bool) -> str:
        targets = dense_targets(samples, step)
        coords: list[str] = []
        for idx, target in enumerate(targets):
            y_value = predict(samples, target)[2] if model else target
            prefix = "M" if idx == 0 else "L"
            coords.append(f"{prefix} {sx(panel_index, target):.2f} {sy(y_value):.2f}")
        return " ".join(coords)

    def circles(panel_index: int, samples: list[ModelSample]) -> str:
        items = []
        for sample in samples:
            x = sx(panel_index, sample.magnitude_abs)
            y = sy(sample.command_product)
            label = (
                f"target={sample.magnitude_abs:.2f}, "
                f"v={sample.rate:.2f}, t={sample.execute_time:.2f}, "
                f"v*t={sample.command_product:.2f}"
            )
            items.append(
                f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4.2" fill="#0f766e">'
                f"<title>{escape_xml(label)}</title></circle>"
            )
        return "\n".join(items)

    def panel(panel_index: int, title: str, samples: list[ModelSample]) -> str:
        left = margin + panel_index * (panel_w + gap)
        right = left + panel_w
        top = margin
        bottom = margin + panel_h
        x_ticks = make_ticks(max_x)
        y_ticks = make_ticks(max_y)
        grid = []
        for tick in x_ticks:
            x = sx(panel_index, tick)
            grid.append(f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{bottom:.2f}" stroke="#e5e7eb"/>')
            grid.append(f'<text x="{x:.2f}" y="{bottom + 22:.2f}" text-anchor="middle">{tick:g}</text>')
        for tick in y_ticks:
            y = sy(tick)
            grid.append(f'<line x1="{left:.2f}" y1="{y:.2f}" x2="{right:.2f}" y2="{y:.2f}" stroke="#e5e7eb"/>')
            grid.append(f'<text x="{left - 10:.2f}" y="{y + 4:.2f}" text-anchor="end">{tick:g}</text>')

        return f"""
<g font-family="Arial, sans-serif" font-size="12" fill="#111827">
  <text x="{(left + right) / 2:.2f}" y="{top - 28:.2f}" text-anchor="middle" font-size="18" font-weight="700">{title}</text>
  <rect x="{left:.2f}" y="{top:.2f}" width="{panel_w:.2f}" height="{panel_h:.2f}" fill="#ffffff" stroke="#d1d5db"/>
  {' '.join(grid)}
  <path d="{path_data(panel_index, samples, False)}" fill="none" stroke="#6b7280" stroke-width="2.2" stroke-dasharray="7 5"/>
  <path d="{path_data(panel_index, samples, True)}" fill="none" stroke="#dc2626" stroke-width="3"/>
  {circles(panel_index, samples)}
  <text x="{(left + right) / 2:.2f}" y="{height - 16:.2f}" text-anchor="middle">target magnitude abs (m)</text>
  <text x="{left - 44:.2f}" y="{(top + bottom) / 2:.2f}" text-anchor="middle" transform="rotate(-90 {left - 44:.2f} {(top + bottom) / 2:.2f})">command product v*t (m)</text>
</g>
"""

    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f9fafb"/>
  <text x="{width / 2:.2f}" y="28" text-anchor="middle" font-family="Arial, sans-serif" font-size="20" font-weight="700">Vigil WBC Move Calibration Model</text>
  {panel(0, "Forward model", forward)}
  {panel(1, "Backward model", backward)}
  <g font-family="Arial, sans-serif" font-size="13" fill="#111827">
    <line x1="{width / 2 - 155}" y1="48" x2="{width / 2 - 120}" y2="48" stroke="#dc2626" stroke-width="3"/>
    <text x="{width / 2 - 112}" y="52">model: magnitude_model = v_model * t_model</text>
    <line x1="{width / 2 + 150}" y1="48" x2="{width / 2 + 185}" y2="48" stroke="#6b7280" stroke-width="2.2" stroke-dasharray="7 5"/>
    <text x="{width / 2 + 193}" y="52">default: expected_magnitude = expected_v * expected_t</text>
  </g>
</svg>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg, encoding="utf-8")


def write_vt_svg_plot(
    path: Path,
    forward: list[ModelSample],
    backward: list[ModelSample],
    min_magnitude: float,
    max_magnitude: float,
    step: float,
) -> None:
    width = 1100
    height = 660
    margin_left = 86
    margin_right = 34
    margin_top = 72
    margin_bottom = 74
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    targets = signed_targets(min_magnitude, max_magnitude, step)
    predicted = [
        (target, *predict_signed(forward, backward, target))
        for target in targets
    ]

    max_t = max([abs(row[2]) for row in predicted] + [sample.execute_time for sample in forward + backward] + [1.0])
    max_v = max([abs(row[1]) for row in predicted] + [sample.rate for sample in forward + backward] + [1.0])
    max_t *= 1.08
    max_v *= 1.18
    min_v = -max_v

    def sx(t_value: float) -> float:
        return margin_left + (t_value / max_t) * plot_w

    def sy(v_value: float) -> float:
        return margin_top + (max_v - v_value) / (max_v - min_v) * plot_h

    def path_for(sign: int) -> str:
        rows = [row for row in predicted if (row[0] > 0.0 and sign > 0) or (row[0] < 0.0 and sign < 0)]
        if sign < 0:
            rows = list(reversed(rows))
        coords = []
        for idx, (_target, signed_rate, execute_time, _signed_product) in enumerate(rows):
            prefix = "M" if idx == 0 else "L"
            coords.append(f"{prefix} {sx(execute_time):.2f} {sy(signed_rate):.2f}")
        return " ".join(coords)

    def training_circles(samples: list[ModelSample], sign: int, color: str) -> str:
        items = []
        for sample in samples:
            signed_rate = sign * sample.rate
            target = sign * sample.magnitude_abs
            label = f"trained target={target:.2f}m, v={signed_rate:.2f}m/s, t={sample.execute_time:.2f}s"
            items.append(
                f'<circle cx="{sx(sample.execute_time):.2f}" cy="{sy(signed_rate):.2f}" '
                f'r="4.2" fill="{color}" stroke="#ffffff" stroke-width="1.5">'
                f"<title>{escape_xml(label)}</title></circle>"
            )
        return "\n".join(items)

    def target_labels() -> str:
        labels = [-10.0, -7.5, -5.0, -2.5, -1.0, -0.5, 0.5, 1.0, 2.5, 5.0, 7.5, 10.0]
        items = []
        for target in labels:
            if target < min_magnitude - 1e-9 or target > max_magnitude + 1e-9:
                continue
            signed_rate, execute_time, _signed_product = predict_signed(forward, backward, target)
            items.append(
                f'<text x="{sx(execute_time) + 7:.2f}" y="{sy(signed_rate) - 6:.2f}" '
                f'font-size="11" fill="#374151">{target:g}m</text>'
            )
        return "\n".join(items)

    x_ticks = make_ticks(max_t)
    v_tick_step = 0.25 if max_v <= 1.5 else 0.5
    y_ticks = []
    current = math.ceil(min_v / v_tick_step) * v_tick_step
    while current <= max_v + 1e-9:
        y_ticks.append(round(current, 6))
        current += v_tick_step

    grid = []
    for tick in x_ticks:
        x = sx(tick)
        grid.append(f'<line x1="{x:.2f}" y1="{margin_top:.2f}" x2="{x:.2f}" y2="{margin_top + plot_h:.2f}" stroke="#e5e7eb"/>')
        grid.append(f'<text x="{x:.2f}" y="{margin_top + plot_h + 24:.2f}" text-anchor="middle">{tick:g}</text>')
    for tick in y_ticks:
        y = sy(tick)
        stroke = "#9ca3af" if abs(tick) < 1e-9 else "#e5e7eb"
        width_attr = "1.5" if abs(tick) < 1e-9 else "1"
        grid.append(f'<line x1="{margin_left:.2f}" y1="{y:.2f}" x2="{margin_left + plot_w:.2f}" y2="{y:.2f}" stroke="{stroke}" stroke-width="{width_attr}"/>')
        grid.append(f'<text x="{margin_left - 12:.2f}" y="{y + 4:.2f}" text-anchor="end">{tick:g}</text>')

    trained_max_abs = min(forward[-1].magnitude_abs, backward[-1].magnitude_abs)
    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f9fafb"/>
  <text x="{width / 2:.2f}" y="32" text-anchor="middle" font-family="Arial, sans-serif" font-size="21" font-weight="700">Vigil WBC Signed Magnitude Model in v-t Space</text>
  <text x="{width / 2:.2f}" y="54" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#6b7280">target magnitude range: {min_magnitude:g}m .. {max_magnitude:g}m; data were trained to about +/-{trained_max_abs:g}m, outside that is extrapolated</text>
  <g font-family="Arial, sans-serif" font-size="12" fill="#111827">
    <rect x="{margin_left:.2f}" y="{margin_top:.2f}" width="{plot_w:.2f}" height="{plot_h:.2f}" fill="#ffffff" stroke="#d1d5db"/>
    {' '.join(grid)}
    <path d="{path_for(1)}" fill="none" stroke="#dc2626" stroke-width="3"/>
    <path d="{path_for(-1)}" fill="none" stroke="#2563eb" stroke-width="3"/>
    {training_circles(forward, 1, "#dc2626")}
    {training_circles(backward, -1, "#2563eb")}
    {target_labels()}
    <text x="{margin_left + plot_w / 2:.2f}" y="{height - 22:.2f}" text-anchor="middle" font-size="14">t_model / execute_time (s)</text>
    <text x="26" y="{margin_top + plot_h / 2:.2f}" text-anchor="middle" font-size="14" transform="rotate(-90 26 {margin_top + plot_h / 2:.2f})">signed v_model (m/s)</text>
    <line x1="{width - 255}" y1="{height - 48}" x2="{width - 220}" y2="{height - 48}" stroke="#dc2626" stroke-width="3"/>
    <text x="{width - 212}" y="{height - 44}">forward targets</text>
    <line x1="{width - 255}" y1="{height - 28}" x2="{width - 220}" y2="{height - 28}" stroke="#2563eb" stroke-width="3"/>
    <text x="{width - 212}" y="{height - 24}">backward targets</text>
  </g>
</svg>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg, encoding="utf-8")


def write_3d_direction_svg(
    path: Path,
    title: str,
    samples: list[ModelSample],
    step: float,
) -> None:
    """Write a pseudo-3D SVG with speed/time on the floor and magnitude vertical."""
    width = 780
    height = 620
    origin = (112.0, 516.0)
    x_vec = (300.0, 0.0)
    y_vec = (160.0, -92.0)
    z_vec = (0.0, -330.0)

    targets = dense_targets(samples, step)
    model_points: list[tuple[float, float, float]] = []
    default_points: list[tuple[float, float, float]] = []
    for target in targets:
        rate, execute_time, command_product = predict(samples, target)
        default_time = target / rate if rate > 1e-9 else 0.0
        model_points.append((rate, execute_time, command_product))
        default_points.append((rate, default_time, target))

    max_v = max([point[0] for point in model_points + default_points] + [1.0]) * 1.12
    max_t = max([point[1] for point in model_points + default_points] + [1.0]) * 1.12
    max_m = max([point[2] for point in model_points + default_points] + [1.0]) * 1.12

    def project(v_value: float, t_value: float, magnitude: float) -> tuple[float, float]:
        x = origin[0] + x_vec[0] * (v_value / max_v) + y_vec[0] * (t_value / max_t) + z_vec[0] * (magnitude / max_m)
        y = origin[1] + x_vec[1] * (v_value / max_v) + y_vec[1] * (t_value / max_t) + z_vec[1] * (magnitude / max_m)
        return x, y

    def line(a: tuple[float, float, float], b: tuple[float, float, float], color: str = "#d1d5db", width_attr: float = 1.0, dash: str = "") -> str:
        x1, y1 = project(*a)
        x2, y2 = project(*b)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" stroke-width="{width_attr}"{dash_attr}/>'

    def text_at(point: tuple[float, float, float], text: str, dx: float = 0.0, dy: float = 0.0, anchor: str = "middle", size: int = 12) -> str:
        x, y = project(*point)
        return f'<text x="{x + dx:.2f}" y="{y + dy:.2f}" text-anchor="{anchor}" font-size="{size}">{escape_xml(text)}</text>'

    def path_for(points: list[tuple[float, float, float]]) -> str:
        coords = []
        for idx, point in enumerate(points):
            x, y = project(*point)
            prefix = "M" if idx == 0 else "L"
            coords.append(f"{prefix} {x:.2f} {y:.2f}")
        return " ".join(coords)

    def circles(points: list[tuple[float, float, float]], color: str, label_prefix: str) -> str:
        items = []
        stride = max(1, int(round(0.25 / step))) if step > 0 else 1
        for idx, (v_value, t_value, magnitude) in enumerate(points):
            if idx % stride != 0 and idx != len(points) - 1:
                continue
            x, y = project(v_value, t_value, magnitude)
            label = f"{label_prefix}: v={v_value:.3f}, t={t_value:.3f}, magnitude={magnitude:.3f}"
            items.append(
                f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3.4" fill="{color}" stroke="#ffffff" stroke-width="1.1">'
                f"<title>{escape_xml(label)}</title></circle>"
            )
        return "\n".join(items)

    grid = []
    for tick in make_ticks(max_v):
        grid.append(line((tick, 0.0, 0.0), (tick, max_t, 0.0), "#e5e7eb"))
        grid.append(text_at((tick, 0.0, 0.0), f"{tick:g}", dy=20.0, size=11))
    for tick in make_ticks(max_t):
        grid.append(line((0.0, tick, 0.0), (max_v, tick, 0.0), "#e5e7eb"))
        grid.append(text_at((0.0, tick, 0.0), f"{tick:g}", dx=-10.0, dy=6.0, anchor="end", size=11))
    for tick in make_ticks(max_m):
        grid.append(line((0.0, 0.0, tick), (max_v, 0.0, tick), "#eef2f7"))
        grid.append(line((0.0, 0.0, tick), (0.0, max_t, tick), "#eef2f7"))
        grid.append(text_at((0.0, 0.0, tick), f"{tick:g}", dx=-12.0, dy=4.0, anchor="end", size=11))

    axis_lines = "\n".join([
        line((0.0, 0.0, 0.0), (max_v, 0.0, 0.0), "#111827", 1.8),
        line((0.0, 0.0, 0.0), (0.0, max_t, 0.0), "#111827", 1.8),
        line((0.0, 0.0, 0.0), (0.0, 0.0, max_m), "#111827", 1.8),
    ])
    labels = "\n".join([
        text_at((max_v, 0.0, 0.0), "v / speed (m/s)", dx=44.0, dy=6.0, anchor="start", size=13),
        text_at((0.0, max_t, 0.0), "t / execute_time (s)", dx=18.0, dy=-8.0, anchor="start", size=13),
        text_at((0.0, 0.0, max_m), "magnitude (m)", dx=-18.0, dy=-10.0, anchor="end", size=13),
    ])

    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f9fafb"/>
  <text x="{width / 2:.2f}" y="32" text-anchor="middle" font-family="Arial, sans-serif" font-size="21" font-weight="700">{escape_xml(title)}</text>
  <text x="{width / 2:.2f}" y="55" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#6b7280">x/y plane: v and t; vertical axis: magnitude. Default uses t=target/v_model; model uses fitted t_model.</text>
  <g font-family="Arial, sans-serif" font-size="12" fill="#111827">
    {' '.join(grid)}
    {axis_lines}
    {labels}
    <path d="{path_for(default_points)}" fill="none" stroke="#6b7280" stroke-width="2.4" stroke-dasharray="7 5"/>
    <path d="{path_for(model_points)}" fill="none" stroke="#dc2626" stroke-width="3.2"/>
    {circles(default_points, "#6b7280", "default")}
    {circles(model_points, "#dc2626", "model")}
    <line x1="{width - 300}" y1="{height - 48}" x2="{width - 262}" y2="{height - 48}" stroke="#dc2626" stroke-width="3.2"/>
    <text x="{width - 252}" y="{height - 44}">model: (v_model, t_model, v_model*t_model)</text>
    <line x1="{width - 300}" y1="{height - 26}" x2="{width - 262}" y2="{height - 26}" stroke="#6b7280" stroke-width="2.4" stroke-dasharray="7 5"/>
    <text x="{width - 252}" y="{height - 22}">default: (v_model, target/v_model, target)</text>
  </g>
</svg>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg, encoding="utf-8")


def make_ticks(max_value: float) -> list[float]:
    if max_value <= 1.5:
        step = 0.25
    elif max_value <= 3.0:
        step = 0.5
    else:
        step = 1.0
    ticks = []
    current = 0.0
    while current <= max_value + 1e-9:
        ticks.append(round(current, 6))
        current += step
    return ticks


def escape_xml(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def print_recommendations(samples: list[ModelSample], direction: str, targets: list[float]) -> None:
    print(f"\n{direction}:")
    for target in targets:
        rate, execute_time, command_product = predict(samples, target)
        print(
            f"  target={target:.2f} m -> "
            f"v_model={rate:.3f} m/s, t_model={execute_time:.3f} s, "
            f"v*t={command_product:.3f} m"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit Vigil WBC move calibration model.")
    parser.add_argument(
        "--source",
        default=None,
        help="Grid summary CSV/XML or grid plan XML. Defaults to latest outputs/vigil_grid_plans/*_summary.csv.",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs/vigil_move_models",
        help="Directory for model JSON, prediction CSV, and SVG plot.",
    )
    parser.add_argument(
        "--plot-step",
        type=float,
        default=0.05,
        help="Target magnitude interval for plotted/predicted model curve.",
    )
    parser.add_argument(
        "--signed-plot-min",
        type=float,
        default=-10.0,
        help="Minimum signed target magnitude for v-t plot/table.",
    )
    parser.add_argument(
        "--signed-plot-max",
        type=float,
        default=10.0,
        help="Maximum signed target magnitude for v-t plot/table.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = latest_summary_file() if args.source is None else resolve_path(args.source)
    if not source.exists():
        raise FileNotFoundError(source)

    rows = read_rows(source)
    forward = build_samples(rows, sign=1)
    backward = build_samples(rows, sign=-1)
    if not forward or not backward:
        raise ValueError("Need both forward and backward calibration rows to fit this model.")

    output_dir = resolve_path(args.output_dir)
    stamp = time.strftime("%y%m%d_%H%M%S")
    stem = f"vigil_move_model_{stamp}"
    model_json = output_dir / f"{stem}.json"
    table_csv = output_dir / f"{stem}_predictions.csv"
    plot_svg = output_dir / f"{stem}_plot.svg"
    vt_table_csv = output_dir / f"{stem}_signed_vt_predictions.csv"
    vt_plot_svg = output_dir / f"{stem}_signed_vt_plot.svg"
    forward_3d_svg = output_dir / f"{stem}_forward_3d_plot.svg"
    backward_3d_svg = output_dir / f"{stem}_backward_3d_plot.svg"

    write_model_json(model_json, source, forward, backward)
    write_prediction_table(table_csv, forward, backward, args.plot_step)
    write_svg_plot(plot_svg, forward, backward, args.plot_step)
    write_signed_vt_table(
        vt_table_csv,
        forward,
        backward,
        args.signed_plot_min,
        args.signed_plot_max,
        args.plot_step,
    )
    write_vt_svg_plot(
        vt_plot_svg,
        forward,
        backward,
        args.signed_plot_min,
        args.signed_plot_max,
        args.plot_step,
    )
    write_3d_direction_svg(forward_3d_svg, "Forward Move Model: v, t, magnitude", forward, args.plot_step)
    write_3d_direction_svg(backward_3d_svg, "Backward Move Model: v, t, magnitude", backward, args.plot_step)

    print(f"Source: {source}")
    print(f"Model JSON: {model_json}")
    print(f"Prediction table: {table_csv}")
    print(f"Plot SVG: {plot_svg}")
    print(f"Signed v-t prediction table: {vt_table_csv}")
    print(f"Signed v-t plot SVG: {vt_plot_svg}")
    print(f"Forward 3D plot SVG: {forward_3d_svg}")
    print(f"Backward 3D plot SVG: {backward_3d_svg}")
    print_recommendations(forward, "forward", [0.25, 0.5, 1.0, 2.0, 5.0])
    print_recommendations(backward, "backward", [0.25, 0.5, 1.0, 2.0, 5.0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
