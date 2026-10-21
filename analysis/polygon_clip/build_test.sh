#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export VCPKG_ROOT="${VCPKG_ROOT:-$(dirname "$ROOT")/vcpkg}"
taskset -c 0-15 cmake --build "$ROOT/build" --parallel 16
cd "$ROOT/build"
python3 - "$ROOT/analysis/polygon_clip/test_polygon_clip.cpp" <<'PY'
import json, pathlib, re, shlex, subprocess, sys

template = next(x for x in json.load(open('compile_commands.json')) if x['file'].endswith('/Src/main.cpp'))
args = shlex.split(template['command'])
args = args[:args.index('-o')] + ['-o', 'test_polygon_clip.o', '-c', sys.argv[1]]
subprocess.run(args, check=True)
section = re.search(r'^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\n(?:  .*\n)+',
                    pathlib.Path('build.ninja').read_text(), re.M).group()
objects = section.splitlines()[0].split()[3:]
objects = objects[:objects.index('||')] if '||' in objects else objects
objects = [x for x in objects if x != '|' and not x.endswith('/Src/main.cpp.o')]
variables = dict(line.strip().split(' = ', 1) for line in section.splitlines() if ' = ' in line)
flags = [x for x in shlex.split(variables.get('LINK_FLAGS', '')) if not x.startswith('-Wl,--dependency-file')]
subprocess.run([args[0], '-g', *flags, 'test_polygon_clip.o', *objects,
                *shlex.split(variables['LINK_LIBRARIES']), '-o', 'test_polygon_clip'], check=True)
print('[ok]', pathlib.Path('test_polygon_clip').resolve())
PY
