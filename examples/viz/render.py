#!/usr/bin/env python3
"""Render a trackinglib example CSV (see AGENTS.md / examples/viz/README.md) into an animated GIF.

One generic renderer drives every example scenario, adapting its content to whichever columns are
present in the CSV, laid out on a 2x2 grid: world frame (top left), tracking frame (bottom left),
and a position NEES plot spanning both rows on the right. The tracking-frame panel shows ground
truth, noisy measurements, the filter's estimated track, and a 1-sigma covariance ellipse
(eigen-decomposition of the P block). The world-frame panel shows the ego vehicle's path and the
target's ground truth in world coordinates; for scenarios without ego motion (no world-frame CSV
columns) the ego is stationary at the origin and the world frame coincides with the tracking frame.

Usage:
    python3 render.py single_linear_track.csv out.gif
    python3 render.py single_nonlinear_track.csv out.gif --show
"""
import argparse
import csv
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


def read_csv(path):
    """Read rows plus the motion-model name from an optional leading `# motion_model=<name>` comment
    line - the only way the generic renderer can know which target model produced a given CSV,
    since that's not otherwise derivable from the column contract."""
    with open(path, newline="") as f:
        first_line = f.readline()
        motion_model = "CV"
        if first_line.startswith("#"):
            key, _, value = first_line.lstrip("#").strip().partition("=")
            if key.strip() == "motion_model" and value.strip():
                motion_model = value.strip()
        else:
            f.seek(0)
        reader = csv.DictReader(f)
        rows = [{k: float(v) for k, v in row.items()} for row in reader]
    return rows, motion_model


def cov_ellipse(pxx, pxy, pyy, n_std=3.0):
    """3-sigma (or n_std-sigma) error ellipse width/height/angle from a 2x2 covariance block."""
    cov = np.array([[pxx, pxy], [pxy, pyy]])
    eigvals, eigvecs = np.linalg.eigh(cov)
    eigvals = np.clip(eigvals, 0.0, None)
    order = eigvals.argsort()[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]
    angle = math.degrees(math.atan2(eigvecs[1, 0], eigvecs[0, 0]))
    width, height = 2.0 * n_std * np.sqrt(eigvals)
    return width, height, angle


def nees(rows):
    """Position NEES e' * P^-1 * e per row, e = ground truth - estimate (tracking frame)."""
    values = []
    for r in rows:
        ex, ey = r["gt_x"] - r["est_x"], r["gt_y"] - r["est_y"]
        p_xx, p_xy, p_yy = r["P_xx"], r["P_xy"], r["P_yy"]
        det = p_xx * p_yy - p_xy * p_xy
        if det <= 0.0:
            values.append(float("nan"))
            continue
        inv_xx, inv_xy, inv_yy = p_yy / det, -p_xy / det, p_xx / det
        values.append(ex * (inv_xx * ex + inv_xy * ey) + ey * (inv_xy * ex + inv_yy * ey))
    return values


def _save_or_show(fig, anim, out_path, show):
    if out_path is not None:
        anim.save(out_path, writer=animation.PillowWriter(fps=8))
    if show:
        plt.show()
    plt.close(fig)


def render(rows, motion_model, out_path, show):
    has_world = "ego_world_x" in rows[0]
    has_polar_meas = "z_range" in rows[0]

    t = [r["t"] for r in rows]
    gt_x = [r["gt_x"] for r in rows]
    gt_y = [r["gt_y"] for r in rows]
    est_x = [r["est_x"] for r in rows]
    est_y = [r["est_y"] for r in rows]
    switch_step = next((i for i, r in enumerate(rows) if r["use_kalman"] > 0.5), None)
    nees_vals = nees(rows)

    # World-frame series: from dedicated CSV columns when present, otherwise the ego is stationary
    # at the origin and the world frame coincides with the tracking frame (no ego motion columns
    # means no ego motion in the scenario).
    if has_world:
        ego_x = [r["ego_world_x"] for r in rows]
        ego_y = [r["ego_world_y"] for r in rows]
        target_x = [r["target_world_x"] for r in rows]
        target_y = [r["target_world_y"] for r in rows]
    else:
        ego_x = [0.0] * len(rows)
        ego_y = [0.0] * len(rows)
        target_x = gt_x
        target_y = gt_y

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.2])
    ax_world = fig.add_subplot(gs[0, 0])
    ax_track = fig.add_subplot(gs[1, 0])
    ax_nees = fig.add_subplot(gs[:, 1])

    # World frame (top left)
    pad = 5.0
    ax_world.set_xlim(min(ego_x + target_x) - pad, max(ego_x + target_x) + pad)
    ax_world.set_ylim(min(ego_y + target_y) - pad, max(ego_y + target_y) + pad)
    ax_world.set_xlabel("X [m]")
    ax_world.set_ylabel("Y [m]")
    ax_world.set_title("World frame")
    ax_world.set_aspect("equal")
    ego_marker_style = {} if has_world else {"marker": "^", "markersize": 8}
    (ego_line,) = ax_world.plot([], [], "k-", label="ego (CTRV)" if has_world else "ego (stationary)", **ego_marker_style)
    (target_line,) = ax_world.plot([], [], "m-", label=f"target ({motion_model})")
    ax_world.legend(loc="upper left", fontsize=8)

    # Tracking frame (bottom left)
    if has_polar_meas:
        # Radar ray approximated from the tracking-frame origin; the sensor mounting offset (a
        # couple of meters) is not part of the CSV contract and is small relative to the scene.
        ray_x = [r["z_range"] * math.cos(r["z_bearing"]) for r in rows]
        ray_y = [r["z_range"] * math.sin(r["z_bearing"]) for r in rows]
        track_x, track_y = gt_x + est_x + ray_x + [0.0], gt_y + est_y + ray_y + [0.0]
    else:
        z_x = [r["z_x"] for r in rows]
        z_y = [r["z_y"] for r in rows]
        track_x, track_y = gt_x + est_x + z_x, gt_y + est_y + z_y

    ax_track.set_xlim(min(track_x) - pad, max(track_x) + pad)
    ax_track.set_ylim(min(track_y) - pad, max(track_y) + pad)
    ax_track.set_xlabel("X [m]")
    ax_track.set_ylabel("Y [m]")
    ax_track.set_title("Tracking frame (ego-centered)")
    ax_track.set_aspect("equal")
    ax_track.scatter([0], [0], c="k", marker="^", s=60, label="ego / sensor")

    (gt_line,) = ax_track.plot([], [], "g-", label="ground truth")
    (est_line,) = ax_track.plot([], [], "b-", label="estimate")
    if has_polar_meas:
        (ray_line,) = ax_track.plot([], [], "r--", linewidth=1, label="radar ray")
    else:
        meas_scatter = ax_track.scatter([], [], c="r", s=15, label="measurement")
    ellipse = Ellipse((0, 0), 0, 0, angle=0, edgecolor="b", facecolor="none", linestyle="--", label="3-sigma cov")
    ax_track.add_patch(ellipse)
    switch_marker = ax_track.scatter([], [], c="orange", s=80, marker="*", label="IF -> KF switch", zorder=5)
    ax_track.legend(loc="upper left", fontsize=7)

    # NEES (right, spans both rows)
    finite_nees = [v for v in nees_vals if math.isfinite(v) and v > 0.0]
    ax_nees.set_xlim(t[0], t[-1])
    if finite_nees:
        ax_nees.set_yscale("log")
        ax_nees.set_ylim(min(finite_nees) * 0.5, max(finite_nees) * 2.0)
    ax_nees.set_xlabel("t [s]")
    ax_nees.set_ylabel("NEES (position)")
    ax_nees.set_title("Position NEES (expect ~2)")
    ax_nees.axhline(2.0, color="gray", linestyle=":", linewidth=1, label="expected")
    (nees_line,) = ax_nees.plot([], [], "b-", label="NEES")
    ax_nees.legend(loc="upper left", fontsize=8)

    def update(frame):
        ego_line.set_data(ego_x[: frame + 1], ego_y[: frame + 1])
        target_line.set_data(target_x[: frame + 1], target_y[: frame + 1])

        gt_line.set_data(gt_x[: frame + 1], gt_y[: frame + 1])
        est_line.set_data(est_x[: frame + 1], est_y[: frame + 1])
        if has_polar_meas:
            ray_line.set_data([0, ray_x[frame]], [0, ray_y[frame]])
        else:
            meas_scatter.set_offsets(np.column_stack([z_x[: frame + 1], z_y[: frame + 1]]))
        r = rows[frame]
        width, height, angle = cov_ellipse(r["P_xx"], r["P_xy"], r["P_yy"])
        ellipse.set_center((r["est_x"], r["est_y"]))
        ellipse.width, ellipse.height, ellipse.angle = width, height, angle
        if switch_step is not None and frame >= switch_step:
            switch_marker.set_offsets([[rows[switch_step]["est_x"], rows[switch_step]["est_y"]]])

        nees_line.set_data(t[: frame + 1], nees_vals[: frame + 1])

        artists = [ego_line, target_line, gt_line, est_line, ellipse, switch_marker, nees_line]
        artists.append(ray_line if has_polar_meas else meas_scatter)
        return artists

    anim = animation.FuncAnimation(fig, update, frames=len(rows), interval=150, blit=False)
    fig.tight_layout()
    _save_or_show(fig, anim, out_path, show)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", help="CSV file written by a trackinglib example")
    parser.add_argument("out_path", nargs="?", default=None, help="Output GIF path (omit with --show for preview only)")
    parser.add_argument("--show", action="store_true", help="Also display the animation live (matplotlib window)")
    args = parser.parse_args()

    if args.out_path is None and not args.show:
        parser.error("either an out_path or --show is required")

    rows, motion_model = read_csv(args.csv_path)
    if not rows:
        parser.error(f"{args.csv_path} contains no data rows")

    render(rows, motion_model, args.out_path, args.show)


if __name__ == "__main__":
    main()
