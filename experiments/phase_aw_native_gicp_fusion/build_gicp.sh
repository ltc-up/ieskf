#!/bin/sh
set -eu
# Existing authorized MIT dependency location; no recursive source copying.
: "${SMALL_GICP_INCLUDE:?set existing upstream/include}"
: "${AW_GICP_BINARY:?set private persistent output binary}"
c++ -O3 -DNDEBUG -std=c++17 -I/usr/include/eigen3 -I"$SMALL_GICP_INCLUDE" "$(dirname "$0")/gicp_service.cpp" -ltbb -o "$AW_GICP_BINARY"
