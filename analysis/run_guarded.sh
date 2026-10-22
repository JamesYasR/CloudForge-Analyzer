#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -eq 0 ]; then
    echo 'Usage: analysis/run_guarded.sh command [arguments...]' >&2
    exit 2
fi
task_root="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$task_root/build"
exec 9>"$task_root/build/.guarded-task.lock"
if ! flock -n 9; then
    echo 'Another guarded task is running; no task was started.' >&2
    exit 1
fi
available_kib=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)
if [ "$available_kib" -lt 10485760 ]; then
    echo 'Available memory below 10 GiB; guarded task was not started.' >&2
    exit 1
fi
systemd-run --user --scope --quiet \
    -p MemoryHigh=6G -p MemoryMax=8G -p MemorySwapMax=256M "$@"
