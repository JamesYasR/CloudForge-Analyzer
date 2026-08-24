set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

set(VCPKG_CMAKE_SYSTEM_NAME Linux)

# CMake 4.x 已移除对 CMake < 3.5 的兼容；vcpkg 里部分旧端口仍写 cmake_minimum_required(3.x)。
# 通过该选项让旧端口在 CMake 4 上也能继续 configure。
set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DCMAKE_POLICY_VERSION_MINIMUM=3.5")

# vcpkg 要求 CXX_FLAGS 与 C_FLAGS 成对出现；这里 C 语言不需要额外头文件。
set(VCPKG_C_FLAGS "${VCPKG_C_FLAGS}")

# Ubuntu 26.04 使用 GCC 15，部分 Qt 内置第三方库（如 glslang）缺少 <cstdint> 的显式包含。
# 全局给所有 C++ 编译增加 -include cstdint，避免 uint32_t 等类型未声明。
set(VCPKG_CXX_FLAGS "${VCPKG_CXX_FLAGS} -include cstdint")

# 2026-08-23 曾因 22GB 无 swap + 高并发 OOM；用户已开启 16G swap，恢复 16 线程并发。
set(VCPKG_CONCURRENCY 16)

# 同时声明 max concurrency，供 vcpkg 工具读取。
set(VCPKG_MAX_CONCURRENCY 16)
