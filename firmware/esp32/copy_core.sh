#!/bin/sh
# Arduino builds every .cpp in the sketch folder, so copy the shared
# firmware core next to the .ino (re-run after changing firmware/core).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cp "$HERE"/../core/*.hpp "$HERE"/../core/*.cpp "$HERE/actuator_node/"
echo "copied firmware/core into $HERE/actuator_node"
