#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export VCPKG_ROOT="${VCPKG_ROOT:-$(dirname "$ROOT")/vcpkg}"
taskset -c 0-15 cmake --build "$ROOT/build" --parallel 2
cd "$ROOT/build"
python3 - "$ROOT/analysis/height_measurement/record_baseline.cpp" <<'PY'
import json, pathlib, re, shlex, subprocess, sys

template = next(x for x in json.load(open('compile_commands.json')) if x['file'].endswith('/Src/main.cpp'))
args = shlex.split(template['command'])
args = args[:args.index('-o')] + ['-fno-access-control', '-o', 'record_height_baseline.o', '-c', sys.argv[1]]
subprocess.run(args, check=True)
section = re.search(r'^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\n(?:  .*\n)+',
                    pathlib.Path('build.ninja').read_text(), re.M).group()
objects = section.splitlines()[0].split()[3:]
objects = objects[:objects.index('||')] if '||' in objects else objects
objects = [x for x in objects if x.endswith('/Src/Measure/MeasureHeight.cpp.o')]
variables = dict(line.strip().split(' = ', 1) for line in section.splitlines() if ' = ' in line)
flags = [x for x in shlex.split(variables.get('LINK_FLAGS', '')) if not x.startswith('-Wl,--dependency-file')]
subprocess.run([args[0], '-g', *flags, 'record_height_baseline.o', *objects,
                *shlex.split(variables['LINK_LIBRARIES']), '-o', 'record_height_baseline'], check=True)
print('[ok]', pathlib.Path('record_height_baseline').resolve())
PY
