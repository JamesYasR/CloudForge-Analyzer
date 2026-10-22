#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export VCPKG_ROOT="${VCPKG_ROOT:-$(dirname "$ROOT")/vcpkg}"
python3 - "$ROOT" <<'PYBASE'
import pathlib, subprocess, sys
out=pathlib.Path(sys.argv[1])/'build/plane_baseline';out.mkdir(exist_ok=True)
for source,dest in [('Inc/Fitting/Fit_Plane.h','BaselineFitPlane.h'),('Src/Fitting/Fit_Plane.cpp','BaselineFitPlane.cpp')]:
    s=subprocess.check_output(['git','-C',sys.argv[1],'show','abcb29d:'+source],text=True).replace('Fitting/Fit_Plane.h','BaselineFitPlane.h').replace('Fit_Plane','BaselineFitPlane')
    path=out/dest
    if not path.exists() or path.read_text()!=s: path.write_text(s)
PYBASE
taskset -c 0-15 cmake --build "$ROOT/build" --parallel 2
cd "$ROOT/build"
python3 - "${1:-$ROOT/analysis/plane_fitting/test_plane_fitting.cpp}" "${2:-test_plane_fitting}" <<'PY'
import json, pathlib, re, shlex, subprocess, sys

template = next(x for x in json.load(open('compile_commands.json')) if x['file'].endswith('/Src/main.cpp'))
args = shlex.split(template['command'])
args = args[:args.index('-o')] + ['-I'+str(pathlib.Path('plane_baseline').resolve()), '-fno-access-control', '-o', sys.argv[2]+'.o', '-c', sys.argv[1]]
subprocess.run(args, check=True)
baseline_args = args[:args.index('-o')] + ['-o', 'plane_baseline/BaselineFitPlane.o', '-c', 'plane_baseline/BaselineFitPlane.cpp']
subprocess.run(baseline_args, check=True)
section = re.search(r'^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\n(?:  .*\n)+',
                    pathlib.Path('build.ninja').read_text(), re.M).group()
objects = section.splitlines()[0].split()[3:]
objects = objects[:objects.index('||')] if '||' in objects else objects
objects = [x for x in objects if x != '|' and not x.endswith('/Src/main.cpp.o')]
variables = dict(line.strip().split(' = ', 1) for line in section.splitlines() if ' = ' in line)
flags = [x for x in shlex.split(variables.get('LINK_FLAGS', '')) if not x.startswith('-Wl,--dependency-file')]
subprocess.run([args[0], '-g', *flags, sys.argv[2]+'.o', 'plane_baseline/BaselineFitPlane.o', *objects,
                *shlex.split(variables['LINK_LIBRARIES']), '-o', sys.argv[2]], check=True)
print('[ok]', pathlib.Path(sys.argv[2]).resolve())
PY
