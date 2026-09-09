# Windows 侧协作约定（致 Ubuntu 侧）

> 2026-09-09，Windows 本地开发环境搭建完成并推送（提交 d18901b、01e86e9）。
> 本文档面向在 Ubuntu 侧开发的 agent/开发者，说明 Windows 侧现状与跨平台协作规则。

## 1. Windows 侧现状一句话

日常构建/调试走 **"CMake 生成 .sln + Visual Studio"** 路线（`build/sln/` 为本机生成物，不入库），
依赖为本机固定套件（`D:\Dev`）：**PCL 1.14.0 / VTK 9.3.0 / Qt 6.7.3**，与 CMakeLists 的 EXACT 约束一致。
旧 Qt VS Tools 工程（sln/vcxproj/props）已从仓库移除（01e86e9），勿重建。

## 2. 协作规则（按重要性排序）

### 2.1 依赖版本升级必须先协调（最重要）

CMakeLists.txt 中 `find_package(... EXACT)` **硬锁版本**：Qt 6.7.3 / PCL 1.14.0 / VTK 9.3.0。
Windows 本机依赖套件是预编译的固定版本，**任何依赖升级都会导致 Windows 侧无法编译**。
若需升级依赖：先在 issue/commit message 中声明 → Windows 侧确认重建环境 → 双侧同步切换。
Ubuntu 侧日常开发（apt/自装的 1.15/9.5/6.10 等版本）若不走 EXACT 校验不受影响——
但仓库 CI 与 Windows 都要求 EXACT，请勿放松版本约束来"适配"本机版本。

### 2.2 CMakePresets.json 为两平台共用文件

- 可**新增** Linux preset（`linux-vcpkg` 之类），与 windows-* 并列互不干扰；
- **勿删改** `windows-*` preset（Windows 本地与 CI 参数记录）；
- `windows-vcpkg-*` 的 hidden 状态是刻意设计，勿改回可见。

### 2.3 CMakeUserPresets.json 不入库

已被 `.gitignore`（395 行）忽略。它是 Qt Creator 自动生成的用户文件，Windows 侧曾因它
混入本机 Qt 路线导致配置错乱。任何机器上都不要 `git add -f` 强制提交它。

### 2.4 CMakeLists.txt 的 if(MSVC) 调试属性段请保留

`VS_DEBUGGER_WORKING_DIRECTORY` / `VS_DEBUGGER_ENVIRONMENT` 两个 target 属性
仅在 MSVC 生成器下生效（Linux 分支不执行、CI 不读取），是 Windows 侧调试的必需配置。

### 2.5 vcpkg 体系（GitHub Actions 打包专用）

`vcpkg.json` / `vcpkg-configuration.json` / `vcpkg-ports/vtk` / `vcpkg-triplets/` /
`.github/workflows/windows-installer.yml` 全部服务于 CI 的 NSIS 安装包流水线。
Windows 本地开发**不依赖** vcpkg（其本地工具 clone 已删除）。改动打包配置时 Windows 侧需知悉。

### 2.6 源码文件正常提交即可

源文件收集为 `GLOB ... CONFIGURE_DEPENDS`：Ubuntu 侧新增/删除 `.cpp/.h` 正常提交，
Windows 侧 pull 后构建时自动重新发现，无需任何登记。

### 2.7 根目录 gen-sln.cmd 是 Windows 专用脚本，勿动

2026-09-09 新增（提交 6658848）。双击即可在 Windows 上生成 `build/sln`
（VS 解决方案，Windows 侧主线流程的一步），实现是纯 cmd 批处理 +
vswhere，只在 Windows 下运行，与 Linux 构建流程零交集。
Ubuntu 侧无需理会，也请勿删除或改动它（cmd 批处理对括号路径极敏感，
脚本内的写法是踩坑后的结果，勿"顺手优化"）。

## 3. Windows 侧详细文档

完整环境说明、构建工作流、踩坑对照表在 Windows 工作机的 `E:\文档\` 下两份文档
（不随仓库分发）：`CloudForgeAnalyzer-Windows构建现状说明(给Agent).md`、
`CloudForgeAnalyzer-Windows日常开发工作流.md`。需要内容同步到本目录时联系 Windows 侧。
