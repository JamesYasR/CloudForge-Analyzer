#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export VCPKG_ROOT="${VCPKG_ROOT:-$(dirname "$ROOT")/vcpkg}"
taskset -c 0-3 cmake --build "$ROOT/build" --parallel 2
cd "$ROOT/build"
python3 - "$ROOT" "${1:-gui}" <<'PY'
import json, pathlib, re, shlex, subprocess, sys

root=pathlib.Path(sys.argv[1])
contracts=sys.argv[2]=='contracts'
if sys.argv[2] not in ('gui','contracts'):
    sys.exit('Usage: build_test.sh [gui|contracts] (run through analysis/run_guarded.sh)')
name='test_clip_contracts' if contracts else 'test_polygon_clip'
source=root/'analysis/polygon_clip'/(name+'.cpp')
template = next(x for x in json.load(open('compile_commands.json')) if x['file'].endswith('/Src/main.cpp'))
args = shlex.split(template['command'])
args = args[:args.index('-o')] + ['-o', name+'.o', '-c', str(source)]
subprocess.run(args, check=True)
section = re.search(r'^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\n(?:  .*\n)+',
                    pathlib.Path('build.ninja').read_text(), re.M).group()
objects = section.splitlines()[0].split()[3:]
objects = objects[:objects.index('||')] if '||' in objects else objects
objects = [x for x in objects if x != '|' and not x.endswith('/Src/main.cpp.o')]
if contracts:
    objects=[x for x in objects if any(x.endswith('/Src/'+suffix+'.o') for suffix in
        ('Application/TaskRunner.cpp','Scene/CloudScene.cpp','UndoRedoManager.cpp'))]
variables = dict(line.strip().split(' = ', 1) for line in section.splitlines() if ' = ' in line)
flags = [x for x in shlex.split(variables.get('LINK_FLAGS', '')) if not x.startswith('-Wl,--dependency-file')]
subprocess.run([args[0], '-g', *flags, name+'.o', *objects,
                *shlex.split(variables['LINK_LIBRARIES']), '-o', name], check=True)
print('[ok]', pathlib.Path(name).resolve())
PY
