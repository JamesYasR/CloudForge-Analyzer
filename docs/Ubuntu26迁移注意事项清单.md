# CloudForge Analyzer 迁移注意事项清单

> 适用路径：`/home/jamesyasr/dsh-workspace/CloudForge-Analyzer`
> 目标：Ubuntu 26.04 LTS 开发，GitHub Actions 产出 Windows x64 安装包。

---

## 0. 版本对齐结论（已按“不改变版本”落地）

你已确认可以接受 vcpkg，并且要求**严格保持原项目版本不变**。因此本仓库现在采用：

| 组件 | 固定版本 | 获取方式 |
| --- | --- | --- |
| PCL | 1.14.0 | vcpkg override：`pcl@1.14.0#3` |
| VTK | 9.3.0 | vcpkg overlay port：`vcpkg-ports/vtk` 拉取 Kitware/VTK `v9.3.0` 官方 tag |
| Qt | 6.7.3 | vcpkg override：`qtbase@6.7.3#1` |

Ubuntu 26.04 的 apt 仓库仍为 PCL 1.15 / VTK 9.5 / Qt 6.10，**不会**再使用这些 apt
版本作为项目依赖；apt 仅用于安装 vcpkg 编译依赖所需的系统开发库。

> 注意：vcpkg 官方 `vtk` 端口是 ParaView 补丁版 `9.3.0-pv5.12.1`，不是原版 VTK 9.3.0。
> 因此这里新增了 `vcpkg-ports/vtk` overlay，使用 `vcpkg_from_git` 直接拉取官方
> `v9.3.0` tag，确保和你在 Windows/本机手工编译的 VTK 9.3.0 版本一致。

## 1. Ubuntu 26.04 LTS + VS Code 环境准备

### 1.1 必装插件

打开 VS Code 后安装 `.vscode/extensions.json` 中推荐的插件：

| 插件 ID | 作用 | 是否必装 |
| --- | --- | --- |
| `ms-vscode.cmake-tools` | CMake 配置/构建/调试入口 | 必装 |
| `ms-vscode.cpptools` | C++ IntelliSense、调试、`c_cpp_properties.json` | 必装 |
| `twxs.cmake` | CMake 脚本高亮 | 必装 |
| `vadimcn.vscode-lldb` | 可选调试器；也可直接用系统 gdb | 可选 |

禁止使用 Qt Creator、CLion 等专属 IDE；项目内不提交任何 Qt Creator 工程文件。

### 1.2 系统依赖安装

apt 只安装 vcpkg 在 Linux 上构建 Qt/VTK/PCL 时需要的系统开发库，不安装 apt 的
PCL/VTK/Qt 包：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake ninja-build git curl zip unzip tar pkg-config gdb bison flex \
  autoconf automake libtool \
  libgl1-mesa-dev libglu1-mesa-dev libegl1-mesa-dev \
  libx11-dev libxkbcommon-dev libxkbcommon-x11-dev libxcb1-dev libxcb-util-dev \
  libxcb-cursor-dev libxcb-icccm4-dev libxcb-keysyms1-dev libxcb-randr0-dev \
  libxcb-render-util0-dev libxcb-shape0-dev libxcb-shm0-dev libxcb-sync-dev \
  libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-xkb-dev libxcb-xv0-dev \
  libxext-dev libxi-dev libxt-dev libxrender-dev libxrandr-dev \
  libfreetype-dev libfontconfig1-dev libdbus-1-dev libharfbuzz-dev \
  libicu-dev libssl-dev libjpeg-dev libpng-dev libtiff-dev libsqlite3-dev \
  libexpat1-dev libx11-xcb-dev libxfixes-dev libxft-dev libxinerama-dev \
  libxkbcommon-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-render-util0-dev \
  libxcb-xinerama0-dev libxcb-xkb-dev libxcb-icccm4-dev libxcb-randr0-dev \
  libxcb-xfixes0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-util-dev \
  libxcb-xv0-dev
```

如果你已经在本机手工安装好 Qt 6.7.3 和 VTK 9.3.0，也可以不运行 vcpkg 的 Linux
构建，只需让 CMake 找到这些安装目录即可。

> 说明：`vcpkg.json` 已将 `qtbase` 的 `default-features` 关闭，只启用
> `concurrent/gui/opengl/widgets`，避免 vcpkg 默认去构建 PostgreSQL 的 `libpq`
> 而要求 `bison`。如果你仍遇到 `Could not find bison`，请先安装 `bison flex`，
> 或删除 `build/vcpkg_installed` 后重新配置。

> 另外：仓库已提供 `vcpkg-triplets/x64-linux.cmake`，为 vcpkg 端口统一注入
> `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`，用于兼容系统 CMake 4.x 对旧端口
> `cmake_minimum_required` 的报错。


### 1.3 VS Code 配置

- `compileCommands` 指向 `${workspaceFolder}/build/compile_commands.json`；
  首次打开后按 `Ctrl+Shift+P -> CMake: Configure` 生成。
- 未执行 CMake Configure 时，`c_cpp_properties.json` 会同时覆盖 vcpkg
  `vcpkg_installed` 与常见 `/usr/local` / `/opt/Qt` 手工安装路径。
- 不要在 `c_cpp_properties.json` 里写 Windows 盘符；Windows 侧 IntelliSense 同样建议
  直接使用 CMake Tools 的 compileCommands。

### 1.4 Linux 构建命令

如果使用 vcpkg 固定版本（推荐，双端一致）：

```bash
# 准备 vcpkg（只需一次）
export VCPKG_ROOT=$HOME/dsh-workspace/vcpkg
git clone https://github.com/microsoft/vcpkg.git "$VCPKG_ROOT"
cd "$VCPKG_ROOT"
git checkout b2cb0da531c2f1f740045bfe7c4dac59f0b2b69c
./bootstrap-vcpkg.sh -disableMetrics
export PATH="$VCPKG_ROOT:$PATH"

# 构建项目（限制并行度，避免内存溢出）
cd /home/jamesyasr/dsh-workspace/CloudForge-Analyzer
export VCPKG_MAX_CONCURRENCY=16
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
  -DVCPKG_TARGET_TRIPLET=x64-linux \
  -DVCPKG_OVERLAY_PORTS="$PWD/vcpkg-ports" \
  -DVCPKG_OVERLAY_TRIPLETS="$PWD/vcpkg-triplets"
cmake --build build --parallel 16
```

如果使用你已手工编译的 Qt 6.7.3 / VTK 9.3.0 / PCL 1.14.0，则不需要 vcpkg，只需要：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_PREFIX_PATH="/opt/Qt/6.7.3/gcc_64;/usr/local;/home/你的用户/Qt/6.7.3/gcc_64" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build --parallel 16
```

注意：CMake 现在使用 `EXACT` 版本校验，找不到精确版本会直接报错，不会静默降级。

---

## 2. 路径大小写适配（已修复示例）

Linux 文件系统大小写敏感，Windows 不敏感。本仓库迁移时已修复以下问题：

1. `Inc/Dialog/<U+200B><U+200B>ParamDialogProtrusion.h` 文件名中包含零宽字符（U+200B），
   CMake/编译器/AUTOMOC 在 Linux 上会出现“找不到文件”的诡异错误。
   **已改为** `Inc/Dialog/ParamDialogProtrusion.h`，并同步修改
   `Inc/Dialog/Dialog.h`、`Inc/PreProcessing/ProtrusionSegmentation.h`、
   `Src/Dialog/ParamDialogProtrusion.cpp` 中的 include。
2. `Src/PreProcessing/CurvatureSegmentation.cpp` 与
   `Src/PreProcessing/ProtrusionSegmentation.cpp` 原先写成
   `#include "preprocessing/..."`（小写 p）。
   **已改为** `#include "PreProcessing/..."`。
3. 旧 `CloudForge Analyzer.vcxproj` 中仍写着
   `Src\Preprocessing\ProtrusionSegmentation.cpp`（小写 p），实际目录是
   `Src\PreProcessing`。CMake 构建入口已不再使用 `.vcxproj`，该文件仅作历史参考。

后续新增代码的检查方法：

```bash
# 检查源码中引用的相对头文件是否与实际文件名大小写完全一致
find Src Inc -type f \( -name '*.h' -o -name '*.cpp' \) \
  -exec grep -Hn '^#include "' {} \; | LC_ALL=C sort > /tmp/includes.txt
```

原则：

- include 字符串必须与 git 中的文件名逐字节一致（含大小写）。
- 禁止在文件名中使用零宽字符、全角字符、空格以外的不可见字符。
- Qt 资源文件 `CloudForgeAnalyzer.qrc` 中 `Icons/*.png` 也必须与实际文件名大小写一致。
- 运行期相对路径 `PCDfiles` 的大小写必须与安装目录一致。

---

## 3. Windows 专属 API 隔离

当前源码中 Windows 专属代码只有 3 处（均在 `Src/CloudForgeAnalyzer.cpp`），模式为：

```cpp
#ifdef _WIN32
    QByteArray localPath = file_name.toLocal8Bit();
    std::string path = localPath.constData();
#else
    std::string path = file_name.toStdString();
#endif
```

要求：

- 禁止直接 include `<windows.h>`，必须使用时应放入 `src/platform/` 或 `Inc/platform/`，
  并通过 `#ifdef _WIN32` / `#ifdef Q_OS_WIN` 隔离。
- 路径转码统一封装为辅助函数，例如：

```cpp
inline std::string toPclPath(const QString& qs) {
#ifdef _WIN32
    return qs.toLocal8Bit().toStdString();
#else
    return qs.toStdString();
#endif
}
```

- 使用 `std::filesystem` 处理路径；不要在业务代码里拼接 `\` 与 `/`。
- 时间、线程、文件锁等优先使用 C++17 标准库与 Qt 跨平台 API。
- MSVC 专属宏（`_CRT_SECURE_NO_WARNINGS`、`NOMINMAX`、`_USE_MATH_DEFINES`）已由
  `CMakeLists.txt` 在 `WIN32` 分支统一定义，不要再散落到源码中。

---

## 4. VTK 版本冲突规避方案

本项目同时使用两类 VTK 入口：

1. `pcl::visualization::PCLVisualizer`；
2. `QVTKOpenGLNativeWidget`（`vtkGUISupportQt`）。

二者必须来自**同一个 VTK 构建**。任何“PCL 自带 VTK + 系统 VTK”或“Qt5 版 VTK +
Qt6 应用”混用都会导致崩溃、符号冲突或运行期找不到 DLL。

### 4.1 Ubuntu 端

- 推荐直接使用 vcpkg，和 Windows 使用同一套 `vcpkg.json` + overlay port。
- 如果使用你手工编译的 VTK 9.3.0，务必确认它是在 Qt 6.7.3 基础上开启：
  `-DVTK_QT_VERSION=6 -DVTK_GROUP_ENABLE_Qt=YES -DVTK_MODULE_ENABLE_VTK_GUISupportQt=YES`
  `-DVTK_MODULE_ENABLE_VTK_RenderingQt=YES -DVTK_MODULE_ENABLE_VTK_ViewsQt=YES`
- 不要同时链接系统 apt 的 `libvtk9` 和手工编译的 VTK 9.3.0，避免两套 VTK。
- 使用 CMake 时只通过 `PCL_LIBRARIES` 和 `VTK::xxx` 目标链接。

### 4.2 Windows/vcpkg 端

`vcpkg.json` 只固定 PCL 和 Qt，VTK 由 overlay port 精确到官方 v9.3.0：

```json
"dependencies": [
  { "name": "pcl",  "features": ["qt", "visualization", "opengl"] },
  { "name": "vtk",  "features": ["qt"] },
  "qtbase"
],
"overrides": [
  { "name": "pcl",    "version": "1.14.0#3" },
  { "name": "qtbase", "version": "6.7.3#1" }
]
```

配套文件：

- `vcpkg-configuration.json` 启用 `./vcpkg-ports/vtk` overlay。
- `vcpkg-ports/vtk/portfile.cmake` 使用 `vcpkg_from_git` 拉取 `v9.3.0` tag。
- 不要再链接原来 all-in-one PCL 的 `$(PCL_ROOT)\3rdParty\VTK`。
- 不要把 `CloudForgeAnalyzer.props` 中手写的数百个 `vtk*-9.3-gd.lib` 搬进 CMake。
- `QVTKOpenGLNativeWidget` 是 Qt6/VTK9 的正确控件；旧的 `QVTKWidget` 只适用于 Qt5。
- GitHub Actions 在 vcpkg 安装后会执行 `Verify exact VTK version from vcpkg` 检查，
  如果出现 `pv5` 或不是 `9.3.0` 会直接失败，防止误装 ParaView 补丁版。

### 4.3 运行时冲突排查

安装后若启动即崩溃：

```powershell
where.exe vtk*.dll Qt6*.dll
dumpbin /dependents CloudForgeAnalyzer.exe | findstr /i "vtk Qt"
```

- 安装目录必须只有一套 `vtk*.dll` 与 `Qt6*.dll`。
- 不要复制 all-in-one PCL 的 `3rdParty\VTK\bin` 与 vcpkg `x64-windows\bin` 两套 DLL。
- NSIS 脚本与 CMake install 均已按“stage 根目录单层全量 DLL”策略收集，避免目录
  分散导致 Windows 搜索不到依赖。

---

## 5. CMake 工程规范

- 根 `CMakeLists.txt` 是唯一构建入口；旧 `.sln/.vcxproj/.props` 仅保留作历史参考。
- 新源码通过 `file(GLOB_RECURSE ... CONFIGURE_DEPENDS)` 自动进入目标，新增文件后
  重新执行 CMake Configure 即可。
- 禁止在 CMake 中写 `C:\`、`/usr/lib/...` 等硬编码路径。
- 平台差异只允许出现在：
  - `if(WIN32)` / `if(UNIX)` 分支；
  - `target_compile_definitions`、`target_link_options` 的生成器表达式或条件块；
  - `vcpkg.json`（仅依赖版本，与路径无关）。
- 新增 Qt 模块时同步修改 `find_package(Qt6 COMPONENTS ...)` 与 `target_link_libraries`。
- 新增 VTK 模块时同步修改 `CFA_VTK_COMPONENTS` 与 `target_link_libraries`。

---

## 6. GitHub Actions / NSIS 打包注意事项

- 工作流文件：`.github/workflows/windows-installer.yml`。
- `vcpkgGitCommitId` 必须与 `vcpkg.json` 的 `builtin-baseline` 完全一致，否则
  版本解析基线会漂移。
- 首次运行会全量编译 Qt/PCL/VTK，耗时很长（数小时）；`VCPKG_BINARY_SOURCES`
  已启用 GitHub 二进制缓存，后续运行命中缓存会大幅加速。
- GitHub Actions 缓存有容量上限；若缓存未命中且耗时超出限制，建议：
  1. 使用私有自托管 Windows runner；
  2. 或预先构建 vcpkg 二进制缓存并上传为 release asset / 自建缓存服务。
- `cmake --install` 会执行 windeployqt 与 vcpkg DLL 全量复制；NSIS 只负责把
  `stage` 目录递归打包，因此修改打包逻辑不需要重编 C++。
- 安装包已包含 MSVC 运行库（windeployqt `--compiler-runtime`）。
- 输出文件名：`CloudForgeAnalyzer-Setup-<版本>.exe`，由 workflow artifact 上传。

---

## 7. 迁移后验收清单

- [ ] Ubuntu 端执行 `cmake -S . -B build && cmake --build build` 成功。
- [ ] `git ls-files | grep -i preprocessing` 无大小写漂移，无零宽字符文件名。
- [ ] `git ls-files | grep -Ei '\.(sln|vcxproj|props)$'` 确认旧 VS 工程不再参与构建。
- [ ] Windows CI 在 `v*` 标签触发后上传 `CloudForgeAnalyzer-Windows-x64-Installer-*.exe`。
- [ ] 在全新 Windows 10/11 x64 虚拟机安装，未装 VC 运行库与 Qt 时可启动。
- [ ] 启动后能加载安装目录下 `PCDfiles/rabbit.pcd`。
- [ ] 点云渲染、PCLVisualizer 交互、STL 导入、OpenMP 并行路径均正常。
