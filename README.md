# Tracking Library

## build
mkdir build && cd build
cmake ..
cmake --build .

## Doxygen
in the root dir run 
doxygen

## create coverage report
mkdir build_cov && cd build_cov
cmake -DCMAKE_BUILD_TYPE=Coverage .. 
cmake --build . --target lcov

## Python binding
in the root dir run 
pip install .