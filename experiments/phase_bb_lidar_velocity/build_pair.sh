#!/bin/sh
set -eu
: "${SMALL_GICP_INCLUDE:?existing authorized MIT upstream include required}"
: "${BB_PAIR_BINARY:?new private output binary required}"
test ! -e "$BB_PAIR_BINARY"
c++ -O3 -DNDEBUG -std=c++17 -I/usr/include/eigen3 -I"$SMALL_GICP_INCLUDE" "$(dirname "$0")/scan_pair_service.cpp" -ltbb -o "$BB_PAIR_BINARY"
