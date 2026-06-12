"""Light exposure metrics for AS7341 spectral data.

This module operates on CSV files produced by the Data_Download/download_script_fixed.py
pipeline (columns: hh, mm, ss, sss, f1..f8, clear, nir, mains_hz, ...).

It computes the following derived quantities:

- UV_risk_score: heuristic proxy for UV-related skin-burn risk, based on
  short-wavelength visible channels (F1–F3) normalized by the total visible
  energy. NOTE: this is NOT a medical-grade UV index because AS7341 does not
  see UV-B. Treat it as a relative risk indicator only.

- uv_dose_cumulative: time integral of UV_risk_score over the recording, using
  the sample timestamps as time base. Units are arbitrary "risk·seconds".

- blue_index: blue-band intensity, here defined as the sum of channels F3+F4,
  which roughly cover the 460–520 nm range depending on AS7341 configuration.

- blue_frac: fraction of visible energy in the blue bands, defined as
  (F3+F4) / (F1+...+F8 + eps).

- blue_weighted_illuminance: proxy for blue-weighted light intensity, defined
  as blue_frac * clear. This scales the blue fraction by overall brightness.

- blue_exposure_cumulative: time integral of blue_weighted_illuminance over
  the whole recording (arbitrary units).

- circadian_dose_cumulative: time integral of blue_weighted_illuminance
  restricted to a configurable "sensitive window" (e.g., evening hours
  before sleep), which is relevant for circadian phase shifting.

The definitions here are intentionally simple and exposed as pure Python so
that they can be tuned later (e.g., different channel weightings or time
windows) without changing the firmware.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import pandas as pd


@dataclass
class CircadianWindow:
    """Time-of-day window for circadian dose integration.

    Times are expressed in hours of local clock time [0, 24).
    For example, start=20.0, end=24.0 integrates exposure from 20:00
    to midnight.
    """

    start_hour: float = 20.0
    end_hour: float = 24.0


def add_time_axis(df: pd.DataFrame) -> pd.DataFrame:
    """Add absolute and relative time axes to DataFrame.

    Expects columns: hh, mm, ss, sss (milliseconds).

    Adds:
    - time_s_abs: seconds of day (float)
    - time_s: time in seconds since first sample (float)
    - time_h: hours of day (float, 0–24)
    """

    df = df.copy()
    t_abs = (
        df["hh"].astype(float) * 3600.0
        + df["mm"].astype(float) * 60.0
        + df["ss"].astype(float)
        + df["sss"].astype(float) / 1000.0
    )
    df["time_s_abs"] = t_abs
    df["time_s"] = t_abs - t_abs.iloc[0]
    df["time_h"] = df["time_s_abs"] / 3600.0
    return df


def compute_uv_risk(df: pd.DataFrame) -> Tuple[pd.Series, pd.Series]:
    """Compute UV_risk_score and its cumulative dose proxy.

    Heuristic definition:
    - Use F1, F2, F3 (shorter visible wavelengths) as a proxy for UV-rich
      conditions.
    - Normalize by sum(F1..F8) to obtain a fraction.
    - Multiply by clear channel to obtain an absolute intensity proxy.

    Returns:
    - uv_risk_score: per-sample scalar
    - uv_dose_cumulative: cumulative trapezoidal integral over time_s
    """

    df = df.copy()
    for col in ["f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "clear"]:
        if col not in df.columns:
            raise KeyError(f"DataFrame missing required column '{col}'")

    short = df["f1"] + df["f2"] + df["f3"]
    total = df[["f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8"]].sum(axis=1)
    eps = 1e-9
    frac_short = short / (total + eps)

    uv_risk_score = frac_short * df["clear"]

    t = df["time_s"].values
    y = uv_risk_score.values
    uv_dose_scalar = np.trapz(y, t)
    # Per-sample cumulative dose (for plotting) via cumulative trapezoid
    # (rectangle rule for simplicity)
    dt = np.diff(t, prepend=t[0])
    uv_dose_cumulative = (y * dt).cumsum()

    uv_risk_score.name = "uv_risk_score"
    uv_dose_cumulative.name = "uv_dose_cumulative"

    return uv_risk_score, uv_dose_cumulative


def compute_blue_metrics(df: pd.DataFrame) -> Tuple[pd.Series, pd.Series, pd.Series]:
    """Compute blue_index, blue_frac, blue_weighted_illuminance.

    Definitions:
    - blue_index = F3 + F4 (blue-ish bands)
    - blue_frac  = (F3 + F4) / sum(F1..F8)
    - blue_weighted_illuminance = blue_frac * clear
    """

    df = df.copy()
    for col in ["f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "clear"]:
        if col not in df.columns:
            raise KeyError(f"DataFrame missing required column '{col}'")

    blue_index = df["f3"] + df["f4"]
    total = df[["f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8"]].sum(axis=1)
    eps = 1e-9
    blue_frac = blue_index / (total + eps)
    blue_weighted_illuminance = blue_frac * df["clear"]

    blue_index.name = "blue_index"
    blue_frac.name = "blue_frac"
    blue_weighted_illuminance.name = "blue_weighted_illuminance"

    return blue_index, blue_frac, blue_weighted_illuminance


def compute_blue_exposure(df: pd.DataFrame, blue_weighted_illuminance: pd.Series) -> pd.Series:
    """Compute cumulative blue-weighted exposure over the recording.

    Uses simple rectangle integration over time_s.
    Returns a per-sample cumulative series.
    """

    t = df["time_s"].values
    y = blue_weighted_illuminance.values
    dt = np.diff(t, prepend=t[0])
    exposure_cumulative = (y * dt).cumsum()
    exposure_cumulative.name = "blue_exposure_cumulative"
    return exposure_cumulative


def compute_circadian_dose(
    df: pd.DataFrame,
    blue_weighted_illuminance: pd.Series,
    window: CircadianWindow,
) -> pd.Series:
    """Compute cumulative circadian-relevant blue-light dose.

    Integration is restricted to samples whose local time-of-day (time_h)
    lies within [window.start_hour, window.end_hour).
    """

    t = df["time_s"].values
    y = blue_weighted_illuminance.values

    # Mask for samples inside the circadian-sensitive window
    h = df["time_h"].values
    mask = (h >= window.start_hour) & (h < window.end_hour)

    dt = np.diff(t, prepend=t[0])
    # Zero out contributions outside the window
    y_eff = np.where(mask, y, 0.0)
    dose_cumulative = (y_eff * dt).cumsum()
    dose_cumulative.name = "circadian_dose_cumulative"
    return dose_cumulative


def process_csv(
    input_csv: str,
    output_csv: str | None = None,
    circadian_window: CircadianWindow | None = None,
) -> pd.DataFrame:
    """Load a CSV, compute all metrics, and optionally write an augmented CSV.

    Parameters
    ----------
    input_csv : str
        Path to CSV produced by download_script_fixed.py.
    output_csv : str or None
        If provided, augmented CSV with new columns is written here.
    circadian_window : CircadianWindow or None
        Time window for circadian dose; if None, defaults to 20:00–24:00.
    """

    df = pd.read_csv(input_csv)
    df = add_time_axis(df)

    uv_risk, uv_dose_cum = compute_uv_risk(df)
    blue_index, blue_frac, blue_w_ill = compute_blue_metrics(df)
    blue_exposure_cum = compute_blue_exposure(df, blue_w_ill)

    if circadian_window is None:
        circadian_window = CircadianWindow()
    circadian_dose_cum = compute_circadian_dose(df, blue_w_ill, circadian_window)

    df[uv_risk.name] = uv_risk
    df[uv_dose_cum.name] = uv_dose_cum
    df[blue_index.name] = blue_index
    df[blue_frac.name] = blue_frac
    df[blue_w_ill.name] = blue_w_ill
    df[blue_exposure_cum.name] = blue_exposure_cum
    df[circadian_dose_cum.name] = circadian_dose_cum

    if output_csv is not None:
        df.to_csv(output_csv, index=False)

    return df


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute UV and blue-light exposure metrics from AS7341 CSV data "
            "exported by download_script_fixed.py."
        )
    )
    parser.add_argument("input_csv", help="Path to input CSV file")
    parser.add_argument(
        "--output-csv",
        help="Path to write augmented CSV. If omitted, no file is written.",
        default=None,
    )
    parser.add_argument(
        "--circadian-start",
        type=float,
        default=20.0,
        help="Start hour (0–24) for circadian-sensitive window (default: 20.0)",
    )
    parser.add_argument(
        "--circadian-end",
        type=float,
        default=24.0,
        help="End hour (0–24) for circadian-sensitive window (default: 24.0)",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    window = CircadianWindow(start_hour=args.circadian_start, end_hour=args.circadian_end)

    df = process_csv(args.input_csv, args.output_csv, circadian_window=window)

    # Print a concise summary of scalar exposures for quick inspection
    total_duration = df["time_s"].iloc[-1] - df["time_s"].iloc[0]
    uv_dose = df["uv_dose_cumulative"].iloc[-1]
    blue_exposure = df["blue_exposure_cumulative"].iloc[-1]
    circadian_dose = df["circadian_dose_cumulative"].iloc[-1]

    print(f"Duration [s]: {total_duration:.1f}")
    print(f"UV dose (arb. units): {uv_dose:.3e}")
    print(f"Blue exposure (arb. units): {blue_exposure:.3e}")
    print(
        f"Circadian dose (arb. units) in window "
        f"[{window.start_hour:.1f}, {window.end_hour:.1f}) h: {circadian_dose:.3e}"
    )


if __name__ == "__main__":  # pragma: no cover
    main()
