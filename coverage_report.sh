rm -rf build_cov && mkdir build_cov && cd build_cov
cmake -DCMAKE_BUILD_TYPE=Coverage .. 
cmake --build . --target lcov
