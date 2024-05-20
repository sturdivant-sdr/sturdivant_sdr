#!/bin/bash

clear
rm -r build
mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=../build \
  # -DENABLE_FAST_MATH=ON ..\
  # -DYAML_CPP_BUILD_TESTS=OFF \
  # -DBUILD_SHARED_LIBS=ON
cmake --build .
# make
# make install
cd ..