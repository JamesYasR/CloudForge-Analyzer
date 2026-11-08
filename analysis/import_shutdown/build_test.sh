#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export VCPKG_ROOT="${VCPKG_ROOT:-$(dirname "$ROOT")/vcpkg}"
# 从仓库根目录用 analysis/run_guarded.sh bash analysis/import_shutdown/build_test.sh 调用。
taskset -c 0-3 cmake --build "$ROOT/build" --parallel 2
cd "$ROOT/build"
python3 - "$ROOT" <<'PY'
import json, pathlib, re, shlex, subprocess, sys
root = pathlib.Path(sys.argv[1])
template = next(x for x in json.load(open('compile_commands.json')) if x['file'].endswith('/Src/main.cpp'))
args = shlex.split(template['command'])
args = args[:args.index('-o')] + ['-fno-access-control', '-o', 'test_import_shutdown.o', '-c',
                               str(root/'analysis/import_shutdown/test_import_shutdown.cpp')]
subprocess.run(args, check=True)
section = re.search(r'^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\n(?:  .*\n)+',
                    pathlib.Path('build.ninja').read_text(), re.M).group()
objects = section.splitlines()[0].split()[3:]
objects = objects[:objects.index('||')] if '||' in objects else objects
objects = [x for x in objects if x != '|' and not x.endswith('/Src/main.cpp.o')]
variables = dict(line.strip().split(' = ', 1) for line in section.splitlines() if ' = ' in line)
flags = [x for x in shlex.split(variables.get('LINK_FLAGS', '')) if not x.startswith('-Wl,--dependency-file')]
subprocess.run([args[0], '-g', *flags, 'test_import_shutdown.o', *objects,
                *shlex.split(variables['LINK_LIBRARIES']), '-o', 'test_import_shutdown'], check=True)
print('[ok]', root/'build/test_import_shutdown')
PY
