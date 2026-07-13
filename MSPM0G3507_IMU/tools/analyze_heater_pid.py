"""Build a reproducible PID-heater diagnostic artifact from monitor CSV data."""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
import pandas as pd


REQUIRED_COLUMNS = {
    "received_at",
    "device_time_ms",
    "phase",
    "temperature_c",
    "target_c",
    "error_c",
    "pwm_percent",
    "feedforward_percent",
    "pid_correction_percent",
    "kp",
    "ki",
    "kd",
}


def first_sustained_time(data: pd.DataFrame, band_c: float) -> float | None:
    outside = np.flatnonzero(data["error_c"].abs().to_numpy() > band_c)
    index = 0 if len(outside) == 0 else int(outside[-1]) + 1
    if index >= len(data):
        return None
    return float(data["time_s"].iloc[index])


def rounded_rows(data: pd.DataFrame, fields: list[str]) -> list[dict]:
    rows = []
    for record in data[fields].to_dict("records"):
        rows.append({
            key: round(float(value), 4) if isinstance(value, (float, np.floating)) else value
            for key, value in record.items()
        })
    return rows


def build_artifact(csv_path: Path) -> dict:
    raw_data = pd.read_csv(csv_path)
    source_sql = """SELECT
    received_at, device_time_ms, phase, temperature_c, target_c, error_c,
    pwm_percent, feedforward_percent, pid_correction_percent, kp, ki, kd
FROM heater_pid_samples
ORDER BY device_time_ms"""
    with sqlite3.connect(":memory:") as connection:
        raw_data.to_sql("heater_pid_samples", connection, index=False)
        data = pd.read_sql_query(source_sql, connection)
    missing = REQUIRED_COLUMNS.difference(data.columns)
    if missing:
        raise ValueError(f"missing required columns: {sorted(missing)}")
    if data[list(REQUIRED_COLUMNS)].isna().any().any():
        raise ValueError("required columns contain null values")

    data["received_at"] = pd.to_datetime(data["received_at"], utc=True)
    data["time_s"] = (
        data["device_time_ms"] - data["device_time_ms"].iloc[0]
    ) / 1000.0

    interval_s = data["device_time_ms"].diff().dropna() / 1000.0
    peak_index = data["temperature_c"].idxmax()
    peak = data.loc[peak_index]
    pid_data = data[data["phase"] == "pid"]
    if pid_data.empty:
        raise ValueError("CSV does not contain a PID phase")
    pid_start = pid_data.iloc[0]

    tail_duration_s = min(60.0, float(data["time_s"].iloc[-1]))
    tail = data[data["time_s"] >= data["time_s"].iloc[-1] - tail_duration_s]
    last_30 = data[data["time_s"] >= data["time_s"].iloc[-1] - 30.0]

    settling_05 = first_sustained_time(data, 0.5)
    settling_025 = first_sustained_time(data, 0.25)
    overshoot_c = float(peak["temperature_c"] - peak["target_c"])
    tail_mae = float(tail["error_c"].abs().mean())
    tail_rms = float(np.sqrt(np.mean(tail["error_c"] ** 2)))
    tail_mean_error = float(tail["error_c"].mean())
    tail_peak_to_peak = float(
        tail["temperature_c"].max() - tail["temperature_c"].min()
    )
    tail_pwm_mean = float(tail["pwm_percent"].mean())
    tail_pwm_std = float(tail["pwm_percent"].std(ddof=0))
    last_30_pwm_mean = float(last_30["pwm_percent"].mean())
    rapid = data[data["phase"] == "rapid"]
    rapid_slope = float(np.polyfit(rapid["time_s"], rapid["temperature_c"], 1)[0])

    summary = [{
        "settling_05_s": settling_05,
        "settling_025_s": settling_025,
        "overshoot_c": overshoot_c,
        "tail_mean_temperature_c": float(tail["temperature_c"].mean()),
        "tail_mean_error_c": tail_mean_error,
        "tail_mae_c": tail_mae,
        "tail_rms_error_c": tail_rms,
        "tail_peak_to_peak_c": tail_peak_to_peak,
        "tail_pwm_mean_percent": tail_pwm_mean,
        "tail_pwm_std_percent": tail_pwm_std,
        "last_30_pwm_mean_percent": last_30_pwm_mean,
        "feedforward_percent": float(data["feedforward_percent"].iloc[0]),
        "kp": float(data["kp"].iloc[0]),
        "ki": float(data["ki"].iloc[0]),
        "kd": float(data["kd"].iloc[0]),
        "sample_count": int(len(data)),
        "duration_s": float(data["time_s"].iloc[-1]),
        "sample_interval_s": float(interval_s.median()),
        "missing_intervals": int((interval_s > 0.2).sum()),
        "duplicate_device_times": int(data["device_time_ms"].duplicated().sum()),
        "rapid_slope_c_per_s": rapid_slope,
    }]

    # Preserve a bounded, evenly spaced response dataset plus all critical points.
    selected = set(range(0, len(data), 4))
    selected.update({0, len(data) - 1, int(peak_index), int(pid_data.index[0])})
    response = data.iloc[sorted(selected)].copy()
    response["phase_code"] = response["phase"].map({"rapid": 1, "pid": 2})

    tail_plot = tail.iloc[::2].copy()
    if tail_plot.index[-1] != tail.index[-1]:
        tail_plot = pd.concat([tail_plot, tail.iloc[[-1]]])

    phase_rows = []
    windows = [
        ("快速加热", rapid),
        ("PID 初始瞬态", data[(data["time_s"] >= pid_start["time_s"]) &
                         (data["time_s"] < settling_025)]),
        ("最后 60 秒", tail),
    ]
    for label, window in windows:
        phase_rows.append({
            "stage": label,
            "time_range_s": f"{window['time_s'].iloc[0]:.1f}–{window['time_s'].iloc[-1]:.1f}",
            "temperature_mean_c": round(float(window["temperature_c"].mean()), 3),
            "temperature_min_c": round(float(window["temperature_c"].min()), 3),
            "temperature_max_c": round(float(window["temperature_c"].max()), 3),
            "error_mae_c": round(float(window["error_c"].abs().mean()), 3),
            "pwm_mean_percent": round(float(window["pwm_percent"].mean()), 2),
        })

    source_id = "heater_pid_csv"
    generated_at = data["received_at"].iloc[-1].isoformat().replace("+00:00", "Z")
    source = {
        "id": source_id,
        "label": csv_path.name,
        "path": csv_path.name,
        "query": {
            "engine": "sqlite",
            "sql": source_sql,
            "description": "Loads all synchronized heater-control samples from the CSV staging table in device-time order.",
            "executed_at": generated_at,
            "language": "sql",
            "tables_used": ["heater_pid_samples"],
            "filters": ["No rows excluded; ORDER BY device_time_ms"],
            "metric_definitions": [
                "Settling time: earliest time after which every remaining sample stays inside the stated absolute-error band",
                "Steady-state MAE: mean absolute target error over the final 60 seconds",
                "Peak-to-peak: maximum minus minimum temperature over the final 60 seconds",
            ],
        },
    }

    title = "Heater PID response diagnosis"
    manifest = {
        "version": 1,
        "surface": "report",
        "title": title,
        "description": "Transient and steady-state diagnosis for the 50 °C heater loop.",
        "generatedAt": generated_at,
        "cards": [
            {
                "id": "settling_card",
                "description": "Time from the first recorded sample until all later samples remain within ±0.5 °C.",
                "dataset": "summary",
                "sourceId": source_id,
                "metrics": [
                    {"label": "±0.5 °C settling", "field": "settling_05_s", "format": "number"},
                    {"label": "Target limit", "field": "settling_target_s", "format": "number"},
                ],
            },
            {
                "id": "steady_error_card",
                "description": "Mean absolute temperature error over the final 60 seconds.",
                "dataset": "summary",
                "sourceId": source_id,
                "metrics": [
                    {"label": "Steady MAE", "field": "tail_mae_c", "format": "number"},
                    {"label": "Required band", "field": "steady_error_limit_c", "format": "number"},
                ],
            },
            {
                "id": "overshoot_card",
                "description": "Maximum temperature above the 50 °C target during the first transient.",
                "dataset": "summary",
                "sourceId": source_id,
                "metrics": [
                    {"label": "Peak overshoot", "field": "overshoot_c", "format": "number"},
                    {"label": "Peak temperature", "field": "peak_temperature_c", "format": "number"},
                ],
            },
            {
                "id": "pwm_card",
                "description": "Average actual PWM over the final 60 seconds versus configured feedforward.",
                "dataset": "summary",
                "sourceId": source_id,
                "metrics": [
                    {"label": "Steady PWM", "field": "tail_pwm_mean_percent", "format": "number"},
                    {"label": "Feedforward", "field": "feedforward_percent", "format": "number"},
                ],
            },
        ],
        "charts": [
            {
                "id": "temperature_response",
                "title": "Temperature response",
                "subtitle": "Full 129 s run; target and measured temperature share the °C scale.",
                "showDescription": True,
                "intent": "trend",
                "question": "How quickly and smoothly did temperature converge to 50 °C?",
                "rationale": "A two-series line chart exposes rise time, overshoot, undershoot, and convergence.",
                "comparisonContext": {"baseline": "50 °C target", "grain": "0.5 s plotted; 0.125 s analyzed", "unit": "°C"},
                "type": "line",
                "dataset": "response",
                "sourceId": source_id,
                "encodings": {
                    "x": {"field": "time_s", "type": "quantitative", "label": "Elapsed time", "unit": "s"},
                    "y": {"field": "temperature_c", "type": "quantitative", "label": "Temperature", "unit": "°C"},
                },
                "referenceLines": [{"axis": "y", "value": 50.0, "label": "Target 50 °C", "color": "neutral", "lineStyle": "dashed"}],
                "palette": {"kind": "sequential", "name": "blue"},
                "layout": "full",
            },
            {
                "id": "pwm_response",
                "title": "PWM response",
                "subtitle": "100% rapid heating transitions to PID control; the line at 25% is configured feedforward.",
                "showDescription": True,
                "intent": "trend",
                "question": "How did actuator output change as the loop entered PID control?",
                "rationale": "A line chart shows the hard transition and the slower equilibrium-power drift.",
                "comparisonContext": {"baseline": "25% feedforward", "grain": "0.5 s plotted; 0.125 s analyzed", "unit": "%"},
                "type": "line",
                "dataset": "response",
                "sourceId": source_id,
                "encodings": {
                    "x": {"field": "time_s", "type": "quantitative", "label": "Elapsed time", "unit": "s"},
                    "y": {"field": "pwm_percent", "type": "quantitative", "label": "PWM duty", "unit": "%"},
                    "tooltip": [
                        {"field": "temperature_c", "type": "quantitative", "label": "Temperature", "unit": "°C"},
                        {"field": "pid_correction_percent", "type": "quantitative", "label": "PID correction", "unit": "%"},
                    ],
                },
                "referenceLines": [{"axis": "y", "value": 25.0, "label": "Feedforward 25%", "color": "neutral", "lineStyle": "dashed"}],
                "palette": {"kind": "sequential", "name": "orange"},
                "layout": "full",
            },
            {
                "id": "steady_error",
                "title": "Steady-state temperature error",
                "subtitle": "Final 60 seconds; reference lines mark the requested ±0.5 °C band.",
                "showDescription": True,
                "intent": "trend",
                "question": "Does the settled loop remain inside ±0.5 °C without meaningful oscillation?",
                "rationale": "An error-over-time line directly tests the requirement and reveals residual drift.",
                "comparisonContext": {"baseline": "zero target error", "grain": "0.25 s plotted; 0.125 s analyzed", "unit": "°C"},
                "type": "line",
                "dataset": "steady_error",
                "sourceId": source_id,
                "encodings": {
                    "x": {"field": "time_s", "type": "quantitative", "label": "Elapsed time", "unit": "s"},
                    "y": {"field": "error_c", "type": "quantitative", "label": "Target error", "unit": "°C"},
                    "tooltip": [{"field": "pwm_percent", "type": "quantitative", "label": "PWM", "unit": "%"}],
                },
                "referenceLines": [
                    {"axis": "y", "value": 0.5, "label": "+0.5 °C", "color": "neutral", "lineStyle": "dashed"},
                    {"axis": "y", "value": 0.0, "label": "Target", "color": "neutral", "lineStyle": "solid"},
                    {"axis": "y", "value": -0.5, "label": "−0.5 °C", "color": "neutral", "lineStyle": "dashed"},
                ],
                "palette": {"kind": "sequential", "name": "blue"},
                "layout": "full",
            },
        ],
        "tables": [
            {
                "id": "stage_summary",
                "title": "Stage summary",
                "subtitle": "Fast heating, initial PID transient, and the final 60-second steady window.",
                "showDescription": True,
                "dataset": "stage_summary",
                "sourceId": source_id,
                "defaultSort": {"field": "stage", "direction": "asc"},
                "density": "spacious",
                "layout": "full",
                "columns": [
                    {"field": "stage", "label": "Stage", "type": "text"},
                    {"field": "time_range_s", "label": "Time (s)", "type": "text"},
                    {"field": "temperature_mean_c", "label": "Mean °C", "format": "number"},
                    {"field": "temperature_min_c", "label": "Min °C", "format": "number"},
                    {"field": "temperature_max_c", "label": "Max °C", "format": "number"},
                    {"field": "error_mae_c", "label": "MAE °C", "format": "number"},
                    {"field": "pwm_mean_percent", "label": "Mean PWM %", "format": "number"},
                ],
            },
        ],
        "sources": [{"id": source_id, "label": csv_path.name, "path": csv_path.name}],
        "blocks": [
            {"id": "title", "type": "markdown", "body": f"# {title}"},
            {
                "id": "technical_summary",
                "type": "markdown",
                "sourceId": source_id,
                "body": (
                    "## Technical summary\n\n"
                    f"**This run already exceeds the ±0.5 °C steady-state requirement.** It entered and then permanently stayed inside ±0.5 °C at {settling_05:.1f} s. Over the final 60 s, mean temperature was {tail['temperature_c'].mean():.3f} °C, MAE was {tail_mae:.3f} °C, and peak-to-peak variation was {tail_peak_to_peak:.3f} °C.\n\n"
                    f"The main remaining weakness is transient, not steady-state control: the first peak reached {peak['temperature_c']:.3f} °C ({overshoot_c:.3f} °C overshoot), followed by one damped undershoot. Keep Kp=10, Ki=0.60, Kd=1.0 as the baseline and optimize the rapid-to-PID transition first."
                ),
            },
            {"id": "headline_metrics", "type": "metric-strip", "cardIds": ["settling_card", "steady_error_card", "overshoot_card", "pwm_card"]},
            {
                "id": "steady_finding",
                "type": "markdown",
                "sourceId": source_id,
                "body": (
                    "## Steady-state accuracy is already comfortably inside ±0.5 °C\n\n"
                    f"All final-60-second samples lie between {tail['temperature_c'].min():.3f} °C and {tail['temperature_c'].max():.3f} °C. The mean error is {tail_mean_error:+.3f} °C and RMS error is {tail_rms:.3f} °C. This is not a 1 °C steady oscillation in this run; the residual variation is only {tail_peak_to_peak:.3f} °C peak-to-peak.\n\n"
                    "So increasing Kp or Ki to chase a smaller steady error is more likely to worsen transient ringing than improve useful accuracy."
                ),
            },
            {"id": "steady_error_chart", "type": "chart", "chartId": "steady_error"},
            {
                "id": "transient_finding",
                "type": "markdown",
                "sourceId": source_id,
                "body": (
                    "## Thermal momentum at the 100%→PID handoff causes the first overshoot\n\n"
                    f"Rapid heating raised temperature at about {rapid_slope:.2f} °C/s. PID began at {pid_start['time_s']:.3f} s and {pid_start['temperature_c']:.3f} °C, but temperature continued rising to {peak['temperature_c']:.3f} °C at {peak['time_s']:.3f} s even while PWM fell as low as {data.loc[peak_index, 'pwm_percent']:.1f}%.\n\n"
                    "That continued rise after output reduction is direct evidence of heater/sensor thermal lag. It is better addressed by ending rapid heating earlier or by a rate-based predictive handoff than by making the PID gains more aggressive."
                ),
            },
            {"id": "temperature_chart", "type": "chart", "chartId": "temperature_response"},
            {
                "id": "feedforward_finding",
                "type": "markdown",
                "sourceId": source_id,
                "body": (
                    "## The 25% feedforward is close, but equilibrium power drifts downward during soak\n\n"
                    f"Average actual PWM over the final 60 s was {tail_pwm_mean:.2f}% versus 25.0% feedforward; over the final 30 s it was {last_30_pwm_mean:.2f}%. The PID therefore needs only a small negative correction after the assembly is fully warm.\n\n"
                    "A 24.5% feedforward is a reasonable later A/B test, but one run is not enough to replace 25.0% confidently. The current integral action already removes this small bias."
                ),
            },
            {"id": "pwm_chart", "type": "chart", "chartId": "pwm_response"},
            {
                "id": "stage_evidence",
                "type": "markdown",
                "body": "## The three operating stages show where tuning effort should go\n\nThe table separates the fast rise, the damped PID transient, and the thermally soaked final minute. It confirms that almost all meaningful error is concentrated immediately after the handoff.",
            },
            {"id": "stage_table", "type": "table", "tableId": "stage_summary"},
            {
                "id": "scope_definitions",
                "type": "markdown",
                "sourceId": source_id,
                "body": (
                    "## Scope and metric definitions\n\n"
                    f"The run contains {len(data)} synchronized samples spanning {data['time_s'].iloc[-1]:.1f} s at an exact median interval of {interval_s.median():.3f} s. Device time has no gaps above 0.2 s, no duplicate timestamps, and no missing values. Settings remained Kp={data['kp'].iloc[0]:.1f}, Ki={data['ki'].iloc[0]:.2f}, Kd={data['kd'].iloc[0]:.1f}, target={data['target_c'].iloc[0]:.1f} °C, and feedforward={data['feedforward_percent'].iloc[0]:.1f}%.\n\n"
                    "Settling time is defined conservatively as the earliest time after which every remaining sample stays inside the stated error band. Steady metrics use the final 60 seconds."
                ),
            },
            {
                "id": "methodology",
                "type": "markdown",
                "body": (
                    "## Method\n\n"
                    "Calculations use all 125 ms device-timestamped rows. The report plots a bounded 0.5 s representation for the full response and 0.25 s for the final minute, while extrema, settling times, means, RMS error, and peak-to-peak values use the complete CSV. No smoothing was used for headline accuracy metrics."
                ),
            },
            {
                "id": "limitations",
                "type": "markdown",
                "body": (
                    "## Limits and robustness\n\n"
                    "This is one 129 s run starting at 30.117 °C. It demonstrates excellent repeatability within the run, but does not establish robustness across cold starts, supply variation, airflow, sensor mounting, or different ambient temperatures. The declining equilibrium PWM suggests a longer thermal soak than the recorded window. Repeat at least three runs and include a 5–10 minute hold before finalizing feedforward."
                ),
            },
            {
                "id": "recommendations",
                "type": "markdown",
                "sourceId": source_id,
                "body": (
                    "## Recommended next experiment\n\n"
                    "1. **Keep Kp=10, Ki=0.60, Kd=1.0 unchanged.** The steady loop already has ample accuracy and damping.\n"
                    "2. **Change only the rapid-to-PID entry temperature from 49.5 °C to 49.0 °C** for the next A/B run. The observed handoff momentum implies this should cut the first overshoot without materially slowing the initial rise.\n"
                    "3. If fixed-threshold behavior varies with ambient conditions, replace it with a predictive handoff such as `temperature + 1.1 s × filtered_dT/dt >= target`; treat 1.1 s as an initial experimental value, not a verified plant constant.\n"
                    "4. Keep 25.0% feedforward for the threshold trial so only one variable changes. After three repeated runs, compare it separately against 24.5%.\n\n"
                    "Acceptance criteria: peak ≤50.3 °C, permanent ±0.5 °C settling by 30 s, final-60-second MAE ≤0.10 °C, and peak-to-peak ≤0.30 °C."
                ),
            },
            {
                "id": "further_questions",
                "type": "markdown",
                "body": (
                    "## Further questions\n\n"
                    "- Does the 49.0 °C handoff reduce overshoot consistently from a true cold start?\n"
                    "- What PWM is required after a full 10-minute soak rather than only two minutes?\n"
                    "- How much do airflow and supply-voltage changes move the required feedforward?"
                ),
            },
        ],
    }

    summary[0].update({
        "settling_target_s": 30.0,
        "steady_error_limit_c": 0.5,
        "peak_temperature_c": float(peak["temperature_c"]),
    })
    fields = [
        "time_s", "phase", "phase_code", "temperature_c", "target_c",
        "error_c", "pwm_percent", "feedforward_percent",
        "pid_correction_percent", "kp", "ki", "kd",
    ]
    steady_fields = [
        "time_s", "temperature_c", "target_c", "error_c", "pwm_percent",
        "feedforward_percent", "pid_correction_percent",
    ]
    return {
        "surface": "report",
        "manifest": manifest,
        "snapshot": {
            "version": 1,
            "generatedAt": generated_at,
            "status": "ready",
            "datasets": {
                "summary": rounded_rows(pd.DataFrame(summary), list(summary[0].keys())),
                "response": rounded_rows(response, fields),
                "steady_error": rounded_rows(tail_plot, steady_fields),
                "stage_summary": phase_rows,
            },
        },
        "sources": [source],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("artifact", type=Path)
    args = parser.parse_args()
    artifact = build_artifact(args.csv)
    args.artifact.parent.mkdir(parents=True, exist_ok=True)
    args.artifact.write_text(
        json.dumps(artifact, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
