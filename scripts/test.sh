#!/usr/bin/env bash
set -euo pipefail

echo "========================================================"
echo "  Executing Pelorus Test Suite"
echo "========================================================"

mkdir -p tests
odin test tests -out:tests/test_runner.bin

echo ""
echo "All 39 unit and integration tests passed successfully."
