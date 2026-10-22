#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
exec "$ROOT/analysis/plane_fitting/build_test.sh" "$ROOT/analysis/plane_fitting/perf_probe.cpp" plane_perf_probe
