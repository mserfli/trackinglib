# Visualizing the tracking examples

`examples/single_linear_object_tracking.cpp`, `examples/single_nonlinear_object_tracking.cpp`, and
`examples/single_nonlinear_figure8_object_tracking.cpp` optionally write a structured CSV (one row
per simulation step) alongside their normal console output. `render.py` reads that CSV and renders
an animated GIF via a single generic renderer that adapts its panels to the CSV columns present: a
tracking-frame panel (ground truth, noisy measurements, the filter's estimated track, and a
1-sigma covariance ellipse) and a position NEES panel are always shown; the nonlinear examples
additionally get a world-frame panel with the ego vehicle's CTRV arc and the target's ground truth.
The figure-8 example shares the exact same CSV column contract as the nonlinear example (just a
different target trajectory and slower ego), so it needs no renderer changes.

This tooling is Python-only and lives entirely outside the C++ build — the library and examples
stay header-only / AUTOSAR-constrained with no new C++ dependency.

## Setup

```bash
pip install matplotlib numpy
```

## Usage

```bash
# Build the examples (from the repo root)
mkdir -p build && cd build
cmake .. && cmake --build . --target single_linear_object_tracking single_nonlinear_object_tracking single_nonlinear_figure8_object_tracking

# Run an example, writing a CSV (path is the first CLI argument; defaults to
# single_linear_track.csv / single_nonlinear_track.csv / single_nonlinear_figure8_track.csv in the
# current directory if omitted)
./examples/single_linear_object_tracking single_linear_track.csv
./examples/single_nonlinear_object_tracking single_nonlinear_track.csv
./examples/single_nonlinear_figure8_object_tracking single_nonlinear_figure8_track.csv

# Render the GIF (auto-detects linear vs. nonlinear from the CSV columns)
python3 ../examples/viz/render.py single_linear_track.csv single_linear_tracking.gif
python3 ../examples/viz/render.py single_nonlinear_track.csv single_nonlinear_tracking.gif
python3 ../examples/viz/render.py single_nonlinear_figure8_track.csv single_nonlinear_figure8_tracking.gif

# Or preview live instead of / in addition to saving a GIF
python3 ../examples/viz/render.py single_linear_track.csv --show
```

## CSV contract

Linear: `step,t,gt_x,gt_y,z_x,z_y,est_x,est_y,est_vx,est_vy,P_xx,P_xy,P_yy,use_kalman`

Nonlinear: `step,t,gt_x,gt_y,est_x,est_y,est_vx,est_vy,P_xx,P_xy,P_yy,use_kalman,ego_world_x,`
`ego_world_y,ego_world_psi,target_world_x,target_world_y,z_range,z_bearing,z_doppler`

`gt_*`/`est_*` are in the tracking frame (ego-centered); `P_xx,P_xy,P_yy` is the position block of
the covariance/information matrix (inverted to a covariance for display while the InformationFilter
is still bootstrapping); `use_kalman` marks the step the sim switched from the InformationFilter to
the KalmanFilter. The nonlinear CSV's radar ray in `render.py` is approximated from the
tracking-frame origin — the sensor's mounting offset is not part of the CSV contract and is small
relative to the plotted scene.
