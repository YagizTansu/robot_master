#!/usr/bin/env python3
"""
live_compare.py — Host-side live comparison dashboard
======================================================
Reads the latest benchmark CSV files from both containers
and shows a side-by-side terminal dashboard.

Usage:
    python3 docker/live_compare.py

    # Custom results directory:
    BENCHMARK_RESULTS_DIR=~/ros2_ws/benchmark_results python3 docker/live_compare.py

Requires: pip install rich   (already in requirements.txt)
Falls back to plain text if rich is not installed.
"""

import os
import csv
import glob
import time
import math

RESULTS_DIR = os.path.expanduser(
    os.environ.get("BENCHMARK_RESULTS_DIR", "~/ros2_ws/benchmark_results")
)
REFRESH_HZ = 2.0


def _latest_csv(pattern: str) -> str | None:
    files = glob.glob(os.path.join(RESULTS_DIR, pattern))
    return max(files, key=os.path.getmtime) if files else None


def _read_last_row(path: str | None) -> dict | None:
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        return rows[-1] if rows else None
    except Exception:
        return None


def _rmse(path: str | None) -> float:
    """Recompute ATE RMSE from all rows in the CSV."""
    if not path or not os.path.exists(path):
        return float("nan")
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            return float("nan")
        errs = [float(r["pos_err_m"]) for r in rows if r.get("pos_err_m")]
        return math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else float("nan")
    except Exception:
        return float("nan")


def _fmt(val: str | None, unit: str = "") -> str:
    if val is None:
        return "  ---  "
    try:
        return f"{float(val):8.4f}{unit}"
    except ValueError:
        return val


def _bar(val: float, good: float, warn: float, width: int = 20) -> str:
    """ASCII progress-bar coloured by threshold."""
    if math.isnan(val):
        return "[" + "-" * width + "]"
    filled = min(int(val / warn * width), width)
    bar = "#" * filled + "-" * (width - filled)
    return f"[{bar}]"


def plain_dashboard() -> None:
    """Fallback plain-text dashboard."""
    while True:
        ekf_csv = _latest_csv("*AMCL*") or _latest_csv("*ekf*") or _latest_csv("*amcl*")
        fgo_csv = _latest_csv("*FGO*") or _latest_csv("*fgo*")

        ekf = _read_last_row(ekf_csv)
        fgo = _read_last_row(fgo_csv)

        ekf_rmse = _rmse(ekf_csv)
        fgo_rmse = _rmse(fgo_csv)

        os.system("clear")
        print("=" * 70)
        print("  LOCALIZATION BENCHMARK — LIVE COMPARISON")
        print(f"  Results dir: {RESULTS_DIR}")
        print("=" * 70)
        print(f"{'Metric':<28} {'AMCL + EKF':>18} {'FGO':>18}")
        print("-" * 70)
        print(f"{'Position Error (m)':<28} {_fmt(ekf.get('pos_err_m') if ekf else None, ' m'):>18} {_fmt(fgo.get('pos_err_m') if fgo else None, ' m'):>18}")
        print(f"{'Yaw Error (deg)':<28} {_fmt(ekf.get('yaw_err_deg') if ekf else None, '°'):>18} {_fmt(fgo.get('yaw_err_deg') if fgo else None, '°'):>18}")
        print(f"{'ATE RMSE (m)':<28} {_fmt(str(ekf_rmse) if not math.isnan(ekf_rmse) else None, ' m'):>18} {_fmt(str(fgo_rmse) if not math.isnan(fgo_rmse) else None, ' m'):>18}")
        print("-" * 70)
        n_ekf = 0
        n_fgo = 0
        if ekf_csv and os.path.exists(ekf_csv):
            with open(ekf_csv) as f:
                n_ekf = sum(1 for _ in csv.DictReader(f))
        if fgo_csv and os.path.exists(fgo_csv):
            with open(fgo_csv) as f:
                n_fgo = sum(1 for _ in csv.DictReader(f))
        print(f"{'Samples collected':<28} {n_ekf:>18} {n_fgo:>18}")
        print(f"{'CSV file':<28} {os.path.basename(ekf_csv) if ekf_csv else 'waiting...':>18} {os.path.basename(fgo_csv) if fgo_csv else 'waiting...':>18}")
        print("=" * 70)
        print("  Refresh: 2 Hz  |  Ctrl-C to exit")
        time.sleep(1.0 / REFRESH_HZ)


def rich_dashboard() -> None:
    from rich.live import Live
    from rich.table import Table
    from rich.console import Console
    from rich import box

    console = Console()

    def build_table() -> Table:
        ekf_csv = _latest_csv("*AMCL*") or _latest_csv("*ekf*") or _latest_csv("*amcl*")
        fgo_csv = _latest_csv("*FGO*") or _latest_csv("*fgo*")

        ekf = _read_last_row(ekf_csv)
        fgo = _read_last_row(fgo_csv)
        ekf_rmse = _rmse(ekf_csv)
        fgo_rmse = _rmse(fgo_csv)

        def colour(val: float, good: float, warn: float) -> str:
            if math.isnan(val):
                return "dim"
            if val <= good:
                return "bold green"
            if val <= warn:
                return "bold yellow"
            return "bold red"

        pos_ekf = float(ekf["pos_err_m"]) if ekf and ekf.get("pos_err_m") else float("nan")
        pos_fgo = float(fgo["pos_err_m"]) if fgo and fgo.get("pos_err_m") else float("nan")
        yaw_ekf = float(ekf["yaw_err_deg"]) if ekf and ekf.get("yaw_err_deg") else float("nan")
        yaw_fgo = float(fgo["yaw_err_deg"]) if fgo and fgo.get("yaw_err_deg") else float("nan")

        t = Table(
            title="[bold cyan]Localization Benchmark — Live Comparison[/]",
            box=box.ROUNDED,
            min_width=72,
        )
        t.add_column("Metric", style="bold white", width=28)
        t.add_column("AMCL + EKF (Domain 1)", justify="right", width=22)
        t.add_column("FGO (Domain 2)", justify="right", width=20)

        t.add_row(
            "Position Error (m)",
            f"[{colour(pos_ekf, 0.05, 0.15)}]{pos_ekf:.4f} m[/]" if not math.isnan(pos_ekf) else "[dim]---[/]",
            f"[{colour(pos_fgo, 0.05, 0.15)}]{pos_fgo:.4f} m[/]" if not math.isnan(pos_fgo) else "[dim]---[/]",
        )
        t.add_row(
            "Yaw Error (°)",
            f"[{colour(yaw_ekf, 2.0, 5.0)}]{yaw_ekf:.2f}°[/]" if not math.isnan(yaw_ekf) else "[dim]---[/]",
            f"[{colour(yaw_fgo, 2.0, 5.0)}]{yaw_fgo:.2f}°[/]" if not math.isnan(yaw_fgo) else "[dim]---[/]",
        )
        t.add_row(
            "ATE RMSE (m)",
            f"[{colour(ekf_rmse, 0.05, 0.20)}]{ekf_rmse:.4f} m[/]" if not math.isnan(ekf_rmse) else "[dim]---[/]",
            f"[{colour(fgo_rmse, 0.05, 0.20)}]{fgo_rmse:.4f} m[/]" if not math.isnan(fgo_rmse) else "[dim]---[/]",
        )

        n_ekf = 0
        n_fgo = 0
        if ekf_csv and os.path.exists(ekf_csv):
            with open(ekf_csv) as f:
                n_ekf = sum(1 for _ in csv.DictReader(f))
        if fgo_csv and os.path.exists(fgo_csv):
            with open(fgo_csv) as f:
                n_fgo = sum(1 for _ in csv.DictReader(f))

        t.add_section()
        t.add_row("Samples", str(n_ekf), str(n_fgo))
        t.add_row(
            "CSV",
            os.path.basename(ekf_csv) if ekf_csv else "[dim]waiting...[/]",
            os.path.basename(fgo_csv) if fgo_csv else "[dim]waiting...[/]",
        )
        return t

    with Live(build_table(), refresh_per_second=REFRESH_HZ, console=console) as live:
        while True:
            time.sleep(1.0 / REFRESH_HZ)
            live.update(build_table())


if __name__ == "__main__":
    os.makedirs(RESULTS_DIR, exist_ok=True)
    try:
        import rich  # noqa: F401
        rich_dashboard()
    except ImportError:
        plain_dashboard()
