#!/usr/bin/env bash
set -euo pipefail

echo "========================================================"
echo "  Building Pelorus Perception Engine"
echo "========================================================"

mkdir -p bin lib

echo "[1/3] Compiling CLI executable (bin/pelorus)..."
odin build src -out:bin/pelorus -o:speed

echo "[2/3] Compiling Benchmark executable (bin/pelorus_bench)..."
odin build bench -out:bin/pelorus_bench -o:speed

echo "[3/3] Compiling C ABI shared library (lib/libpelorus.dylib)..."
odin build src/c_api -build-mode:shared -out:lib/libpelorus.dylib -o:speed

echo ""
echo "Build complete. All artifacts generated in bin/ and lib/."
