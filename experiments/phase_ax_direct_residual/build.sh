#!/bin/sh
set -eu
: "${SMALL_GICP_INCLUDE:?existing MIT small_gicp include}"
: "${AX_BINARY:?private output binary}"
c++ -O3 -DNDEBUG -std=c++17 -I/usr/include/eigen3 -I"$SMALL_GICP_INCLUDE" "$(dirname "$0")/direct_service.cpp" -ltbb -o "$AX_BINARY"
