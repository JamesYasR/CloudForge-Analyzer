#!/usr/bin/env bash
# build_test.sh —— 焊前装配阶差/间隙 无界面验证程序构建脚本(不进 CMakeLists)
#
# 与 analysis/cylinder_patch/build_test.sh 同一套路: 复用主程序 build/ 里已配置好的
# 编译/链接参数(从 compile_commands.json 与 build.ninja 解析), 先同步工程目标文件,
# 再编译本目录的测试 .cpp 并链接。
#
# 用法:
#   analysis/weld_preparation/build_test.sh            # 构建 test_weld_prep
#
# 注意(铁律): 改了 Inc/Measure/MeasureWeldPreparation.h 之类头文件后, 必须重跑本脚本,
# 否则测试 .o 与工程 .o 的 ABI 不一致, 会读出垃圾数据。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD="$ROOT/build"
SRC="$ROOT/analysis/weld_preparation/test_weld_prep.cpp"
OUT="test_weld_prep"

mkdir -p "$BUILD"

echo "[0/3] 同步主程序目标文件 (ninja CloudForgeAnalyzer)..."
ninja -C "$BUILD" CloudForgeAnalyzer >/dev/null || { echo "主程序编译失败"; exit 1; }
cd "$BUILD"

python3 - "$SRC" "$OUT" <<'PY'
import json, re, subprocess, sys, shlex, os

src, out = sys.argv[1], sys.argv[2]

cc = json.load(open('compile_commands.json'))
tpl = next(e for e in cc if e['file'].endswith('Src/Measure/MeasureWeldPreparation.cpp'))
args = shlex.split(tpl['command'])
o = args.index('-o')
args = args[:o] + ['-o', f'{out}.o', '-c', src]

print('[compile]', ' '.join(args[:6]), '...')
rv = subprocess.call(args)
if rv != 0:
    sys.exit('编译失败')

ninja = open('build.ninja', encoding='utf-8').read()
m = re.search('(^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\\n(?:  .*\\n)+)', ninja, re.M)
if not m:
    sys.exit('无法解析 build.ninja 链接段')
vars_ = {}
for line in m.group(1).splitlines():
    if ' = ' in line:
        k, v = line.strip().split(' = ', 1)
        vars_[k] = v
objects = m.group(0).splitlines()[0].split()[3:]
if '||' in objects:
    objects = objects[:objects.index('||')]
objects = [x for x in objects if x != '|']
libs = shlex.split(vars_.get('LINK_LIBRARIES', ''))
link_flags = shlex.split(vars_.get('LINK_FLAGS', ''))
link_flags = [f for f in link_flags if not f.startswith('-Wl,--dependency-file')]

drop = {'CMakeFiles/CloudForgeAnalyzer.dir/Src/main.cpp.o',
        'CMakeFiles/CloudForgeAnalyzer.dir/Src/CloudForgeAnalyzer.cpp.o'}
objects = [o for o in objects if o not in drop]
cmd = ['/usr/bin/c++', '-g'] + link_flags + [f'{out}.o'] + objects + libs + ['-o', out]
print('[link] 目标文件数 =', len(objects))
rv = subprocess.call(cmd)
if rv != 0:
    sys.exit('链接失败')
print('[ok]', os.path.join(os.getcwd(), out))
PY
