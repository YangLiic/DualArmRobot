#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ ! -f /usr/include/eigen3/Eigen/Dense ]]; then
  echo "Missing Eigen headers. Install: sudo apt-get install libeigen3-dev"
  exit 1
fi

if [[ ! -f /usr/include/boost/property_tree/ptree.hpp ]]; then
  echo "Missing Boost headers. Install: sudo apt-get install libboost-dev"
  exit 1
fi

if [[ ! -f /usr/include/ncurses.h ]]; then
  echo "Missing ncurses headers. Install: sudo apt-get install libncurses-dev"
  exit 1
fi

g++ "$SCRIPT_DIR/sdk_smoke_test.cpp" \
  -I"$SDK_DIR/include" \
  -I/usr/include/eigen3 \
  -L"$SDK_DIR/usrlib" \
  -lHumanoidArms \
  -lcontrolcan \
  -lncurses \
  -ltinfo \
  -o "$SCRIPT_DIR/sdk_smoke_test"

echo "Built: $SCRIPT_DIR/sdk_smoke_test"
