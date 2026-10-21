# Windows CI 的 MSYS2 工具包修复

最近一次原始失败记录：
https://github.com/JamesYasR/CloudForge-Analyzer/actions/runs/36426139180

失败发生在 vcpkg 安装 libiconv 时：构建工具包
`msys2-libiconv-1.17-1-x86_64.pkg.tar.zst` 在所有镜像返回 404。
固定 baseline 中另有同类退役包，原工作流只替换了其中四项。

`patch-msys.ps1` 根据 `msys-packages.json` 同时替换退役归档的 URL 与
SHA512；若固定 baseline 中的原 URL/哈希不匹配，立即失败。清单中的
替代归档均从 https://repo.msys2.org/msys/x86_64/ 下载并计算 SHA512，
优先选择仍可下载的相邻版本，保留原来已经使用的 runtime/file 版本。
这里只修复 MSYS2 构建工具，不改变 vcpkg baseline、应用依赖版本、
CMakeLists.txt 或 Windows 本地预设。

工作流使用 vcpkg `files` 二进制缓存，并通过 actions/cache 恢复/保存。
依赖安装失败时也保存已经完成的端口；缓存恢复后仍由 vcpkg 按 ABI
匹配验证。不使用需要特殊运行时令牌的 `x-gha` provider。

维护时须同时更新清单中的 URL 和 SHA512，先验证实际归档，再运行
完整 Windows 安装包工作流。禁止关闭哈希校验或放松 Qt 6.7.3 /
PCL 1.14.0 / VTK 9.3.0 的精确版本约束。

## 依赖缓存命中（2026-10-21）

此前第二轮虽然恢复了 574 MB 的文件缓存，vcpkg 却显示 `Restored 0 package(s)`。
两轮 runner 的 PowerShell 分别为 7.6.7 和 7.6.6；锁定的 vcpkg-tool
2024-11-12 在 `commands.build.cpp` 中将 PowerShell 和 CMake 版本计入每个
包的 ABI，因此系统工具漂移会使所有包失效。

CI 现在显式选择 Windows 2025/VS2026 系列、MSVC 14.44.35207 与 SDK
10.0.26100.0，并设置 `VCPKG_FORCE_DOWNLOADED_BINARIES=1`，由已锁定的
vcpkg baseline 下载固定工具（包括 CMake/Ninja/PowerShell）。应用和依赖
统一使用 v143，应用配置不再隐式重新执行依赖安装。Qt/PCL/VTK 版本不变。

缓存 key 包含编译器哈希、固定工具版本与归档哈希、依赖清单/overlay/triplet
及 MSYS2 修复输入；不包含业务源码、README、run id。仅在依赖安装成功且
没有精确命中时保存完整缓存，避免每次运行重复保存相同归档。

验证必须分两轮：

1. 普通运行填充缓存并完成 Windows 编译、打包。
2. 同一版本以 `cache_only=true` 手动运行；vcpkg 加
   `--only-binarycaching`，缺失的依赖直接失败，不允许靠源码重建“通过”。
   同时要求 `restored > 0`、`built = 0`，并继续完成应用编译、打包。

每轮 summary 中的 `[DEPENDENCY_CACHE]` 记录实际恢复数、源码构建数及
依赖安装耗时。`dependency-cache-diagnostics` artifact 保存完整安装日志、
工具指纹及 `*vcpkg_abi_info.txt`，便于精确比较失效原因。缓存被清退、
依赖或工具版本变化后需要重新填充；禁止绕过 ABI 兼容性检查强行复用。

## 长期依赖备份与缓存缺失恢复

Actions 缓存受保留期和容量淘汰策略影响。依赖安装成功后，工作流会将
`vcpkg-bincache` 中的二进制包归档到专用 prerelease `ci-dependency-cache`，
不标为最新应用版本，也不触发 `v*` 安装包标签构建。每套工具链/依赖 key
各自保留 `.tar`、`.tar.sha256` 和带来源/工具链信息的 `.json` 附件。
已有备份不覆盖，后续运行只检查是否存在，避免重复上传。

恢复顺序：Actions 精确缓存 → 同 key 的 Release 归档（校验 SHA256）→
vcpkg 按 ABI 复用。Release 附件不受 Actions 缓存七天未访问清理规则影响，
但不是不可删除的存储：手动删除 Release/附件/仓库会丢失备份。请保留
`ci-dependency-cache`；更换依赖或编译工具需要另建兼容版本。固定工具集在
未来 runner 中若被移除，构建会明确失败，不能以不兼容工具强行复用。

验证长期恢复：先正常运行一次备份现有依赖；随后同时设置
`restore_from_release=true` 和 `cache_only=true` 手动构建。前者跳过
Actions 缓存读取，要求 Release 归档存在并下载校验成功，后者禁止依赖
源码重编。验收须 Release 恢复成功、126 包复用、0 包源码构建及整个
Windows 安装包流水线成功，不能只依据附件上传成功下结论。

本脚本当前限制单个归档小于 2 GiB（本项目约 575 MiB）；超过时明确失败，
需拆分归档后再上传。GitHub Release 附件限制与 Actions 缓存规则不同，
不应通过周期性运行两小时依赖构建来“保活”。
