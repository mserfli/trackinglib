# Tracking Library

## build
mkdir build && cd build
cmake ..
cmake --build .

## Doxygen
in the root dir run 
doxygen

## create coverage report
mkdir build && cd build
cmake ..
cmake --build . --target lcov

## Python binding
in the root dir run 
pip install .