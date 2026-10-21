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
