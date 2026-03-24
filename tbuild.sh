#!/bin/bash

# Only clean if argument is passed
if [ "$1" == "clean" ]; then
  rm -rf build
fi

mkdir -p build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..

make
