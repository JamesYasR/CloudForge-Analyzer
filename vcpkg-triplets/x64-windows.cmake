set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)

# CMake 4.x 已移除对 CMake < 3.5 的兼容；vcpkg 里部分旧端口仍写 cmake_minimum_required(3.x)。
# 通过该选项让旧端口在 CMake 4 上也能继续 configure。
set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DCMAKE_POLICY_VERSION_MINIMUM=3.5")

# 限制 vcpkg 并行编译数为 16，避免 Windows runner 内存不足。
set(VCPKG_CONCURRENCY 16)

# 同时声明 max concurrency，供 vcpkg 工具读取。
set(VCPKG_MAX_CONCURRENCY 16)
