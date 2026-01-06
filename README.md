# Tracking Library

## build
mkdir build && cd build
cmake ..
cmake --build .

## Doxygen
in the root dir run 
doxygen

## create coverage report
run `report_coverage.sh` and open `build_cov/coverage/index.html` in a webbrowser

## Python binding
in the root dir run 
pip install .