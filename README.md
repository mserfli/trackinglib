# Tracking Library

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

Academic C++ header-only library for object tracking using Kalman filter variants.

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## under construction 

### build
mkdir build && cd build
cmake ..
cmake --build .

### Doxygen
in the root dir run
doxygen

### create coverage report
run `report_coverage.sh` and open `build_cov/coverage/index.html` in a webbrowser

### Python binding
in the root dir run
pip install .