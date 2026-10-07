#!/bin/bash
# Day one on the Jetson Nano: builds everything this repo runs there and tests it.
#   1. splat for the GPU, and its unit tests (the GPU ones have to run)
#   2. a small synthetic scan rendered on the GPU and the CPU, and a short training run
#   3. the ROS 2 packages in their container, and their tests
# Run jetson/nano_setup.sh once first. The first run downloads the images and compiles
# everything; later runs only redo what changed. It stops at the first step that fails.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)

step() { printf '\n=== %s\n\n' "$*"; }

step "splat: build for the GPU"
"$HERE/splat.sh" build
step "splat: unit tests"
"$HERE/splat.sh" test
step "splat: a small synthetic dataset (~/linesplat/synth), rendered on the GPU and the CPU"
"$HERE/splat.sh" splat_synth synth --preset small
"$HERE/splat.sh" splat_render compare synth
step "splat: 300 training steps on the GPU (~/linesplat/synth/train)"
"$HERE/splat.sh" splat_train synth synth/train --iterations 300
step "ROS 2: build"
"$HERE/ros.sh" colcon build
step "ROS 2: tests"
# colcon test reports failures through colcon test-result, so take the exit code from both.
# shellcheck disable=SC2016  # expanded in the container
"$HERE/ros.sh" bash -c 'colcon test --executor sequential; s=$?; colcon test-result --verbose && exit $s'
step "All built and tested"
