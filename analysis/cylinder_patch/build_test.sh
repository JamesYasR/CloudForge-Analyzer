#!/usr/bin/env bash
# build_test.sh -- 无界面测试程序构建脚本(不进 CMakeLists)
#
# 背景: test_cyl_pothole.cpp / test_async_progress.cpp 是无界面验证程序,
# 不加入 CMakeLists.txt(铁律: 不改 CMakeLists)。这里复用主程序 build/ 里
# 已经配置好的编译/链接参数(从 build.ninja 解析), 编译测试 .cpp 并链接。
#
# 用法:
#   analysis/cylinder_patch/build_test.sh                 # 构建 test_cyl_pothole
#   analysis/cylinder_patch/build_test.sh base            # 构建 test_cyl_pothole_base(冻结基线 .o)
#   analysis/cylinder_patch/build_test.sh async           # 构建 test_async_progress
#
# 说明:
#   - 编译用的 -D/-I/--std 等直接取自主程序某个 TUs 的编译命令, 保证 ABI 一致;
#   - 链接复用主程序的目标文件与库, 但剔除 Src/main.cpp.o 与 Src/CloudForgeAnalyzer.cpp.o
#     (它们提供自己的 main / 大量 GUI 符号, 与测试程序冲突; 测试程序不引用其中的符号)。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD="$ROOT/build"
which="${1:-pothole}"

case "$which" in
  pothole) SRC="$ROOT/analysis/cylinder_patch/test_cyl_pothole.cpp"; OUT="test_cyl_pothole" ;;
  base)    SRC="$ROOT/analysis/cylinder_patch/test_cyl_pothole.cpp"; OUT="test_cyl_pothole_base" ;;
  async)   SRC="$ROOT/analysis/cylinder_patch/test_async_progress.cpp"; OUT="test_async_progress" ;;
  *) echo "未知目标: $which (可选 pothole|base|async)"; exit 2 ;;
esac

mkdir -p "$BUILD"

# ---- 0. 先确保主程序目标文件是最新的 ----
# 改了头文件(如 Inc/Measure/MeasurePothole.h)后, 工程 .o 必须重编译,
# 否则测试程序与 .o 的 ABI/结构体布局不一致, 会读出全 0 的错数据.
# (铁律: 改了头文件后测试 .o 必须重编译再链接)
echo "[0/3] 同步主程序目标文件 (ninja CloudForgeAnalyzer)..."
ninja -C "$BUILD" CloudForgeAnalyzer >/dev/null || { echo "主程序编译失败"; exit 1; }
cd "$BUILD"

python3 - "$SRC" "$OUT" <<'PY'
import json, re, subprocess, sys, shlex, os

src, out = sys.argv[1], sys.argv[2]
root = os.path.dirname(os.getcwd())

# ---- 1. 取主程序某 TU 的编译命令, 复用其编译参数 ----
cc = json.load(open('compile_commands.json'))
tpl = next(e for e in cc if e['file'].endswith('Src/Measure/MeasurePothole.cpp'))
args = shlex.split(tpl['command'])
# 去掉 -o XXX -c SRC, 换成我们的
o = args.index('-o')
args = args[:o] + ['-o', f'{out}.o', '-c', src]

print('[compile]', ' '.join(args[:6]), '...')
rv = subprocess.call(args)
if rv != 0:
    sys.exit('编译失败')

# ---- 2. 解析 build.ninja 里的链接变量 ----
ninja = open('build.ninja', encoding='utf-8').read()
m = re.search('(^build CloudForgeAnalyzer: CXX_EXECUTABLE_LINKER.*\\n(?:  .*\\n)+)', ninja, re.M)
if not m:
    sys.exit('无法解析 build.ninja 链接段')
vars_ = {}
for line in m.group(1).splitlines():
    if ' = ' in line:
        k, v = line.strip().split(' = ', 1)
        vars_[k] = v
# 第一行是 "build <target>: <rule> <objs...> | <implicit> || <order-only>",
# 其中 || 之后是 order-only 依赖(autogen 等 phony 目标), 不能进链接命令.
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
