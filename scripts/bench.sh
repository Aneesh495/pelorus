#!/usr/bin/env bash
set -euo pipefail

echo "========================================================"
echo "  Executing Pelorus Performance Benchmark Suite"
echo "========================================================"

mkdir -p bin
if [ ! -f "bin/pelorus_bench" ]; then
    echo "Compiling benchmark executable..."
    odin build bench -out:bin/pelorus_bench -o:speed
fi

./bin/pelorus_bench
