# =============================================================================
# CloudForgeAnalyzer 跨平台 CMake 工程
#
# 目标环境：
#   Linux  : Ubuntu 26.04 LTS + vcpkg 固定版本（或用户手工安装的相同版本）
#   Windows: Windows 10/11 + VS2022(MSVC v143) + vcpkg(x64-windows)
#
# 依赖版本策略（严格不改变原项目版本）：
#   PCL  = 1.14.0
#   VTK  = 9.3.0（使用 vcpkg overlay port 拉取 Kitware/VTK v9.3.0 官方 tag）
#   Qt6  = 6.7.3
# 因此 CMake 使用 EXACT 查找，apt 的 1.15/9.5/6.10 不会通过。
#
# 说明：
#   1. 不使用任何绝对路径、盘符或 VS 专属配置。
#   2. Qt 的 uic/moc/rcc 通过 CMAKE_AUTOUIC/AUTOMOC/AUTORCC 完成。
#   3. Windows 安装包运行时由 qt_generate_deploy_app_script(windeployqt)
#      与 vcpkg bin 目录全量 DLL 收集共同保证，NSIS 脚本只负责打包 stage 目录。
# =============================================================================
cmake_minimum_required(VERSION 3.24)

project(CloudForgeAnalyzer
    VERSION 1.0.0
    DESCRIPTION "Point cloud analysis GUI based on PCL/VTK/Qt6"
    LANGUAGES CXX
)

# -----------------------------------------------------------------------------
# 全局配置
# -----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # 供 VS Code c_cpp_properties 使用

include(GNUInstallDirs)

option(CFA_INSTALL_SAMPLE_DATA "Install the PCDfiles sample data directory" ON)

# Qt 自动处理：moc/uic/rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}")

# -----------------------------------------------------------------------------
# 依赖查找：全部使用 CMake package，Windows/Linux 自动区分
# -----------------------------------------------------------------------------
# Qt6：本项目使用 Widgets + OpenGL 控件 + QtConcurrent
find_package(Qt6 6.7.3 EXACT REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    OpenGL
    OpenGLWidgets
    Concurrent
)
if(COMMAND qt_standard_project_setup)
    qt_standard_project_setup()
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# OpenMP：项目多个文件使用 omp parallel for / critical
# MSVC 新版对 reduction(min/max) 要求 /openmp:llvm，旧版 /openmp 会报 C7660。
if(MSVC)
    set(OpenMP_RUNTIME_MSVC llvm)
endif()
find_package(OpenMP REQUIRED COMPONENTS CXX)

# 先找 PCL：PCLConfig 会自动定位其编译时使用的 VTK，避免混用两套 VTK。
find_package(PCL 1.14.0 EXACT REQUIRED COMPONENTS
    common
    features
    filters
    io
    kdtree
    sample_consensus
    search
    segmentation
    surface
    visualization
)

# 显式声明 VTK 组件。
# GUISupportQt 是 QVTKOpenGLNativeWidget 的宿主模块；其余为源码直接/间接使用。
# 注意：Ubuntu 26.04 apt 默认 VTK 9.5、Windows vcpkg 固定 9.3，组件名均兼容。
set(CFA_VTK_COMPONENTS
    CommonColor
    CommonComputationalGeometry
    CommonCore
    CommonDataModel
    CommonMath
    CommonTransforms
    FiltersCore
    FiltersGeneral
    FiltersSources
    GUISupportQt
    InteractionStyle
    IOGeometry
    RenderingAnnotation
    RenderingCore
    RenderingFreeType
    RenderingOpenGL2
    RenderingQt
    RenderingUI
)
find_package(VTK 9.3.0 EXACT REQUIRED COMPONENTS ${CFA_VTK_COMPONENTS})

message(STATUS "CloudForgeAnalyzer dependency summary")
message(STATUS "  Qt  : ${Qt6_VERSION}  (required 6.7.3 EXACT)")
message(STATUS "  PCL : ${PCL_VERSION}  (required 1.14.0 EXACT)")
message(STATUS "  VTK : ${VTK_VERSION}  (required 9.3.0 EXACT)")

# -----------------------------------------------------------------------------
# 源码收集
# -----------------------------------------------------------------------------
file(GLOB_RECURSE CFA_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
)
file(GLOB_RECURSE CFA_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc/*.h"
)

# qt_add_executable 会自动处理 Qt entry point 与平台 GUI 子系统；
# 保留 add_executable 分支仅为兼容较老的 Qt 6.0/6.1。
if(COMMAND qt_add_executable)
    qt_add_executable(${PROJECT_NAME}
        ${CFA_SOURCES}
        ${CFA_HEADERS}
        "${CMAKE_CURRENT_SOURCE_DIR}/CloudForgeAnalyzer.ui"
        "${CMAKE_CURRENT_SOURCE_DIR}/CloudForgeAnalyzer.qrc"
    )
else()
    add_executable(${PROJECT_NAME}
        ${CFA_SOURCES}
        ${CFA_HEADERS}
        "${CMAKE_CURRENT_SOURCE_DIR}/CloudForgeAnalyzer.ui"
        "${CMAKE_CURRENT_SOURCE_DIR}/CloudForgeAnalyzer.qrc"
    )
endif()

if(WIN32)
    # 原生 Windows GUI 程序：无控制台窗口
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src"
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Concurrent
    ${PCL_LIBRARIES}
    VTK::GUISupportQt
    VTK::RenderingQt
    VTK::RenderingOpenGL2
    VTK::RenderingUI
    VTK::InteractionStyle
    VTK::RenderingCore
    VTK::RenderingAnnotation
    VTK::RenderingFreeType
    VTK::FiltersSources
    VTK::FiltersGeneral
    VTK::FiltersCore
    VTK::IOGeometry
    VTK::CommonColor
    VTK::CommonComputationalGeometry
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::CommonMath
    VTK::CommonTransforms
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
)

# -----------------------------------------------------------------------------
# 平台编译/链接参数（不硬编码路径，只按平台与编译器区分）
# -----------------------------------------------------------------------------
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /utf-8 /bigobj /permissive- /MP)
    # 与原 vcxproj 保持一致的 16 MiB 栈保留
    target_link_options(${PROJECT_NAME} PRIVATE "/STACK:16777216,1048576")
endif()

if(WIN32)
    # 与原 CloudForgeAnalyzer.props 保持一致的 Windows 宏
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _USE_MATH_DEFINES
        _CRT_SECURE_NO_WARNINGS
        UNICODE
        _UNICODE
        BOOST_USE_WINDOWS_H
    )
endif()

# -----------------------------------------------------------------------------
# 安装规则
# Windows：exe 与全部 DLL 平铺在安装根目录，便于 NSIS 用 File /r 一键收集。
# Linux  ：遵循 GNUInstallDirs。
# -----------------------------------------------------------------------------
if(WIN32)
    set(CFA_RUNTIME_INSTALL_DIR ".")
else()
    set(CFA_RUNTIME_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
endif()

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION "${CFA_RUNTIME_INSTALL_DIR}"
    BUNDLE  DESTINATION "."
)

if(CFA_INSTALL_SAMPLE_DATA)
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/PCDfiles"
        DESTINATION "."
    )
endif()

# Windows + vcpkg：全量收集 vcpkg 安装树中的 release DLL。
# 精确依赖裁剪可使用 dumpbin 递归实现，但为了保证“可独立运行”，这里采用全量策略。
if(WIN32 AND DEFINED VCPKG_INSTALLED_DIR AND EXISTS "${VCPKG_INSTALLED_DIR}")
    if(NOT DEFINED VCPKG_TARGET_TRIPLET)
        set(VCPKG_TARGET_TRIPLET "x64-windows")
    endif()
    set(_cfa_vcpkg_bin_dir "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
    if(EXISTS "${_cfa_vcpkg_bin_dir}")
        message(STATUS "  vcpkg runtime DLL dir: ${_cfa_vcpkg_bin_dir}")
        install(DIRECTORY "${_cfa_vcpkg_bin_dir}/"
            DESTINATION "${CFA_RUNTIME_INSTALL_DIR}"
            FILES_MATCHING
            PATTERN "*.dll"
        )
    else()
        message(WARNING "VCPKG_INSTALLED_DIR is set but no bin dir found: ${_cfa_vcpkg_bin_dir}")
    endif()
endif()

# Qt 官方 Windows 部署脚本：自动执行 windeployqt，含 Qt 插件、Qt DLL 与 MSVC 运行库。
if(WIN32 AND COMMAND qt_generate_deploy_app_script)
    set(_cfa_deploy_script "${CMAKE_CURRENT_BINARY_DIR}/deploy_${PROJECT_NAME}.cmake")
    qt_generate_deploy_app_script(
        TARGET ${PROJECT_NAME}
        OUTPUT_SCRIPT "${_cfa_deploy_script}"
        NO_UNSUPPORTED_PLATFORM_ERROR
    )
    # VS18/CMake4.4 下该脚本可能未实际生成；只有存在时才安装，缺失时由 workflow 手动 windeployqt 兜底。
    if(EXISTS "${_cfa_deploy_script}")
        install(SCRIPT "${_cfa_deploy_script}")
    else()
        message(WARNING "qt_generate_deploy_app_script did not produce ${_cfa_deploy_script}; workflow will run windeployqt manually.")
    endif()
elseif(WIN32)
    message(WARNING "qt_generate_deploy_app_script not available; run windeployqt manually before packaging.")
endif()
