#!/usr/bin/env bash

export BUILD_TYPE=${1:-Debug}
echo "CMAKE_BULD_TYPE: ${BUILD_TYPE}"
cmake -H. -B_build -DCMAKE_BUILD_TYPE=${BUILD_TYPE} || exit 1
cmake --build _build --target cbag_polygon_test -- -j 8 || exit 1
./_build/tests/cbag_polygon_test
