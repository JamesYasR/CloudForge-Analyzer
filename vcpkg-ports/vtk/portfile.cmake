set(VTK_SHORT_VERSION 9.3)
if(NOT VCPKG_TARGET_IS_WINDOWS)
    message(WARNING "You will need to install Xorg dependencies to build vtk:\napt-get install libxt-dev\n")
endif()

set(VCPKG_POLICY_SKIP_ABSOLUTE_PATHS_CHECK enabled)

# =============================================================================
# Clone exact upstream Kitware/VTK v9.3.0 release tag and apply only the
# patches required by vcpkg's external dependencies and package layout.
# This intentionally does NOT use the ParaView-patched v9.3.x commit.
# =============================================================================
vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL "https://github.com/Kitware/VTK.git"
    # vcpkg 要求 REF 必须是 commit SHA，tag 用 FETCH_REF 指定。
    REF "13793e66202ad0a1b1492c9a2cb8a23cc3d67ac4"
    FETCH_REF "v9.3.0"
    PATCHES
        FindLZMA.patch
        FindLZ4.patch
        fast-float.patch
        fix-exprtk.patch
        fix-using-hdf5.patch
        FindExpat.patch
        jsoncpp.patch
        pegtl.patch
        no-libharu-for-ioexport.patch
        fix-octree-gcc15.patch
        remove-prefix-changes.patch
)

# =============================================================================
#Overwrite outdated modules if they have not been patched:
file(COPY "${CURRENT_PORT_DIR}/FindHDF5.cmake" DESTINATION "${SOURCE_PATH}/CMake/patches/99") # due to usage of targets in netcdf-c

# =============================================================================

# =============================================================================
# Options:
# 本 overlay 只保留 Qt6 / OpenGL / SQL 三个 feature，对应原项目的
# vtkGUISupportQt、RenderingQt、RenderingOpenGL2、IOSQL 等模块。
# =============================================================================
set(ADDITIONAL_OPTIONS "")

vcpkg_check_features(OUT_FEATURE_OPTIONS VTK_FEATURE_OPTIONS
    FEATURES
        "qt"          VTK_GROUP_ENABLE_Qt
        "qt"          VTK_MODULE_ENABLE_VTK_GUISupportQt
        "qt"          VTK_MODULE_ENABLE_VTK_GUISupportQtSQL
        "qt"          VTK_MODULE_ENABLE_VTK_RenderingQt
        "qt"          VTK_MODULE_ENABLE_VTK_ViewsQt
        "sql"         VTK_MODULE_ENABLE_VTK_IOSQL
        "opengl"      VTK_MODULE_ENABLE_VTK_ImagingOpenGL2
        "opengl"      VTK_MODULE_ENABLE_VTK_RenderingGL2PSOpenGL2
        "opengl"      VTK_MODULE_ENABLE_VTK_RenderingOpenGL2
        "opengl"      VTK_MODULE_ENABLE_VTK_RenderingVolumeOpenGL2
        "opengl"      VTK_MODULE_ENABLE_VTK_opengl
)

vcpkg_check_features(OUT_FEATURE_OPTIONS PACKAGE_FEATURE_OPTIONS
  FEATURES
    "sql" CMAKE_REQUIRE_FIND_PACKAGE_SQLite3
  INVERTED_FEATURES
    "sql" CMAKE_DISABLE_FIND_PACKAGE_SQLite3
    "libtheora" CMAKE_DISABLE_FIND_PACKAGE_THEORA
    "libharu" CMAKE_DISABLE_FIND_PACKAGE_LibHaru
    "cgns" CMAKE_DISABLE_FIND_PACKAGE_CGNS
    "seacas" CMAKE_DISABLE_FIND_PACKAGE_SEACASIoss
    "seacas" CMAKE_DISABLE_FIND_PACKAGE_SEACASExodus
    "proj" CMAKE_DISABLE_FIND_PACKAGE_PROJ
    "netcdf" CMAKE_DISABLE_FIND_PACKAGE_NetCDF
)

# Replace common value to vtk value
list(TRANSFORM VTK_FEATURE_OPTIONS REPLACE "=ON" "=YES")
list(TRANSFORM VTK_FEATURE_OPTIONS REPLACE "=OFF" "=DONT_WANT")

if("qt" IN_LIST FEATURES AND NOT EXISTS "${CURRENT_HOST_INSTALLED_DIR}/tools/Qt6/bin/qmlplugindump${VCPKG_HOST_EXECUTABLE_SUFFIX}")
    list(APPEND VTK_FEATURE_OPTIONS -DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick=NO)
endif()
if("qt" IN_LIST FEATURES)
    file(READ "${CURRENT_INSTALLED_DIR}/share/qtbase/vcpkg_abi_info.txt" qtbase_abi_info)
    if(qtbase_abi_info MATCHES "(^|;)gles2(;|$)")
        message(FATAL_ERROR "VTK assumes qt to be build with desktop opengl. As such trying to build vtk with qt using GLES will fail.")
    endif()
endif()

if("opengl" IN_LIST FEATURES)
    list(APPEND ADDITIONAL_OPTIONS
        -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES
        -DVTK_MODULE_ENABLE_VTK_RenderingLICOpenGL2=YES
        -DVTK_MODULE_ENABLE_VTK_RenderingAnnotation=YES
        -DVTK_MODULE_ENABLE_VTK_FiltersParallelDIY2=YES
    )
endif()

list(APPEND ADDITIONAL_OPTIONS
    -DVTK_USE_TK=OFF
    -DVTK_FORBID_DOWNLOADS=ON
    # 项目不使用 Ogg/Theora/Movie，禁用这些 VTK 模块，避免要求 OGG 外部库。
    -DVTK_MODULE_ENABLE_VTK_ogg=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_theora=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOOggTheora=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOMovie=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOVideo=DONT_WANT
    # 禁用 StandAlone 组里不需要的第三方/IO 模块，避免它们强制查找外部库。
    -DVTK_MODULE_ENABLE_VTK_netcdf=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IONetCDF=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOMINC=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_exodusII=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOExodus=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_ioss=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOIOSS=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_cgns=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOCGNSReader=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_proj=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_libproj=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOCesium3DTiles=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_GeovisCore=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_libharu=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOExport=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_IOExportPDF=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_RenderingVtkJS=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_DomainsChemistry=DONT_WANT
    -DVTK_MODULE_ENABLE_VTK_DomainsChemistryOpenGL2=DONT_WANT
)

# =============================================================================
# Configure & Install



# We set all libraries to "system" and explicitly list the ones that should use embedded copies
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${VTK_FEATURE_OPTIONS}
        ${PACKAGE_FEATURE_OPTIONS}
        -DBUILD_TESTING=OFF
        -DVTK_BUILD_TESTING=OFF
        -DVTK_BUILD_EXAMPLES=OFF
        -DVTK_ENABLE_REMOTE_MODULES=OFF
        # VTK groups to enable
        -DVTK_GROUP_ENABLE_StandAlone=YES
        -DVTK_GROUP_ENABLE_Rendering=YES
        -DVTK_GROUP_ENABLE_Views=YES
        # Disable deps not in VCPKG
        -DVTK_USE_TK=OFF # TCL/TK currently not included in vcpkg
        # Select modules / groups to install
        -DVTK_USE_EXTERNAL:BOOL=ON
        -DVTK_MODULE_USE_EXTERNAL_VTK_gl2ps:BOOL=OFF # Not yet in VCPKG
        -DVTK_MODULE_USE_EXTERNAL_VTK_token:BOOL=OFF # Not yet in VCPKG
        #-DVTK_MODULE_ENABLE_VTK_jsoncpp=YES
        ${ADDITIONAL_OPTIONS}
        -DVTK_DEBUG_MODULE_ALL=ON
        -DVTK_DEBUG_MODULE=ON
        -DVTK_QT_VERSION=6
        -DCMAKE_INSTALL_QMLDIR:PATH=qml
        -DVCPKG_HOST_TRIPLET=${_HOST_TRIPLET}
        -DCMAKE_FIND_PACKAGE_TARGETS_GLOBAL=ON # Due to Qt6::Platform not being found on Linux platform
    MAYBE_UNUSED_VARIABLES
        VTK_MODULE_ENABLE_VTK_PythonContext2D # Guarded by a conditional
        VTK_MODULE_ENABLE_VTK_GUISupportMFC # only windows
        VTK_QT_VERSION # Only with Qt
        CMAKE_INSTALL_QMLDIR
        # When working properly these should be unused
        CMAKE_DISABLE_FIND_PACKAGE_CGNS
        CMAKE_DISABLE_FIND_PACKAGE_LibHaru
        CMAKE_DISABLE_FIND_PACKAGE_NetCDF
        CMAKE_DISABLE_FIND_PACKAGE_PROJ
        CMAKE_DISABLE_FIND_PACKAGE_SEACASExodus
        CMAKE_DISABLE_FIND_PACKAGE_SEACASIoss
        CMAKE_DISABLE_FIND_PACKAGE_SQLite3
        CMAKE_DISABLE_FIND_PACKAGE_THEORA
        CMAKE_REQUIRE_FIND_PACKAGE_CGNS
        CMAKE_REQUIRE_FIND_PACKAGE_LibHaru
        CMAKE_REQUIRE_FIND_PACKAGE_NetCDF
        CMAKE_REQUIRE_FIND_PACKAGE_PROJ
        CMAKE_REQUIRE_FIND_PACKAGE_SEACASExodus
        CMAKE_REQUIRE_FIND_PACKAGE_SEACASIoss
        CMAKE_REQUIRE_FIND_PACKAGE_SQLite3
        CMAKE_REQUIRE_FIND_PACKAGE_THEORA

)

vcpkg_cmake_install()
vcpkg_copy_pdbs()

# =============================================================================
# Fixup target files
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/vtk-${VTK_SHORT_VERSION})

# =============================================================================
# Clean-up other directories

# Delete the debug binary TOOL_NAME that is not required
function(_vtk_remove_debug_tool TOOL_NAME)
    set(filename "${CURRENT_PACKAGES_DIR}/debug/bin/${TOOL_NAME}${VCPKG_TARGET_EXECUTABLE_SUFFIX}")
    if(EXISTS "${filename}")
        file(REMOVE "${filename}")
    endif()
    set(filename "${CURRENT_PACKAGES_DIR}/debug/bin/${TOOL_NAME}d${VCPKG_TARGET_EXECUTABLE_SUFFIX}")
    if(EXISTS "${filename}")
        file(REMOVE "${filename}")
    endif()
    if (NOT VCPKG_BUILD_TYPE OR VCPKG_BUILD_TYPE STREQUAL debug)
        # we also have to bend the lines referencing the tools in VTKTargets-debug.cmake
        # to make them point to the release version of the tools
        file(READ "${CURRENT_PACKAGES_DIR}/share/vtk/VTK-targets-debug.cmake" VTK_TARGETS_CONTENT_DEBUG)
        string(REPLACE "debug/bin/${TOOL_NAME}" "tools/vtk/${TOOL_NAME}" VTK_TARGETS_CONTENT_DEBUG "${VTK_TARGETS_CONTENT_DEBUG}")
        string(REPLACE "tools/vtk/${TOOL_NAME}d" "tools/vtk/${TOOL_NAME}" VTK_TARGETS_CONTENT_DEBUG "${VTK_TARGETS_CONTENT_DEBUG}")
        file(WRITE "${CURRENT_PACKAGES_DIR}/share/vtk/VTK-targets-debug.cmake" "${VTK_TARGETS_CONTENT_DEBUG}")
    endif()
endfunction()

# Move the release binary TOOL_NAME from bin to tools
function(_vtk_move_release_tool TOOL_NAME)
    set(old_filename "${CURRENT_PACKAGES_DIR}/bin/${TOOL_NAME}${VCPKG_TARGET_EXECUTABLE_SUFFIX}")
    if(EXISTS "${old_filename}")
        file(INSTALL "${old_filename}" DESTINATION "${CURRENT_PACKAGES_DIR}/tools/vtk" USE_SOURCE_PERMISSIONS)
        file(REMOVE "${old_filename}")
    endif()

    if (NOT VCPKG_BUILD_TYPE OR VCPKG_BUILD_TYPE STREQUAL release)
        # we also have to bend the lines referencing the tools in VTKTargets-release.cmake
        # to make them point to the tool folder
        file(READ "${CURRENT_PACKAGES_DIR}/share/vtk/VTK-targets-release.cmake" VTK_TARGETS_CONTENT_RELEASE)
        string(REPLACE "bin/${TOOL_NAME}" "tools/vtk/${TOOL_NAME}" VTK_TARGETS_CONTENT_RELEASE "${VTK_TARGETS_CONTENT_RELEASE}")
        file(WRITE "${CURRENT_PACKAGES_DIR}/share/vtk/VTK-targets-release.cmake" "${VTK_TARGETS_CONTENT_RELEASE}")
    endif()
endfunction()

set(VTK_TOOLS
    vtkEncodeString-${VTK_SHORT_VERSION}
    vtkHashSource-${VTK_SHORT_VERSION}
    vtkWrapTclInit-${VTK_SHORT_VERSION}
    vtkWrapTcl-${VTK_SHORT_VERSION}
    vtkWrapPythonInit-${VTK_SHORT_VERSION}
    vtkWrapPython-${VTK_SHORT_VERSION}
    vtkWrapJava-${VTK_SHORT_VERSION}
    vtkWrapHierarchy-${VTK_SHORT_VERSION}
    vtkParseJava-${VTK_SHORT_VERSION}
    vtkParseOGLExt-${VTK_SHORT_VERSION}
    vtkProbeOpenGLVersion-${VTK_SHORT_VERSION}
    vtkTestOpenGLVersion-${VTK_SHORT_VERSION}
    vtkpython
    pvtkpython
)
# TODO: Replace with vcpkg_copy_tools if known which tools are built with which feature
# or add and option to vcpkg_copy_tools which does not require the tool to be present
foreach(TOOL_NAME IN LISTS VTK_TOOLS)
    _vtk_remove_debug_tool("${TOOL_NAME}")
    _vtk_move_release_tool("${TOOL_NAME}")
endforeach()

if(EXISTS "${CURRENT_PACKAGES_DIR}/bin/vtktoken-9.3.dll" AND VCPKG_LIBRARY_LINKAGE STREQUAL "static")
  # vendored "token" library can be only build as a shared library
  set(VCPKG_POLICY_DLLS_IN_STATIC_LIBRARY enabled)
elseif(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
  file(REMOVE_RECURSE
    "${CURRENT_PACKAGES_DIR}/bin"
    "${CURRENT_PACKAGES_DIR}/debug/bin")
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")

vcpkg_copy_tool_dependencies("${CURRENT_PACKAGES_DIR}/tools/vtk")

if(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
    if(EXISTS "${CURRENT_BUILDTREES_DIR}/${TARGET_TRIPLET}-rel/CMakeFiles/vtkpythonmodules/static_python") #python headers
        file(GLOB_RECURSE STATIC_PYTHON_FILES "${CURRENT_BUILDTREES_DIR}/${TARGET_TRIPLET}-rel/CMakeFiles/*/static_python/*.h")
        file(INSTALL ${STATIC_PYTHON_FILES} DESTINATION "${CURRENT_PACKAGES_DIR}/include/vtk-${VTK_SHORT_VERSION}")
    endif()
endif()

#remove one get_filename_component(_vtk_module_import_prefix "${_vtk_module_import_prefix}" DIRECTORY) from vtk-prefix.cmake and VTK-vtk-module-properties and vtk-python.cmake
set(filenames_fix_prefix vtk-prefix VTK-vtk-module-properties vtk-python)
foreach(name IN LISTS filenames_fix_prefix)
if(EXISTS "${CURRENT_PACKAGES_DIR}/share/vtk/${name}.cmake")
    file(READ "${CURRENT_PACKAGES_DIR}/share/vtk/${name}.cmake" _contents)
    string(REPLACE
[[set(_vtk_module_import_prefix "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(_vtk_module_import_prefix "${_vtk_module_import_prefix}" DIRECTORY)]]
[[set(_vtk_module_import_prefix "${CMAKE_CURRENT_LIST_DIR}")]] _contents "${_contents}")
    file(WRITE "${CURRENT_PACKAGES_DIR}/share/vtk/${name}.cmake" "${_contents}")
else()
    debug_message("FILE:${CURRENT_PACKAGES_DIR}/share/vtk/${name}.cmake does not exist! No prefix correction!")
endif()
endforeach()

# Use vcpkg provided find method
file(REMOVE "${CURRENT_PACKAGES_DIR}/share/${PORT}/FindEXPAT.cmake")

file(RENAME "${CURRENT_PACKAGES_DIR}/share/licenses" "${CURRENT_PACKAGES_DIR}/share/${PORT}/licenses")

if(EXISTS "${CURRENT_PACKAGES_DIR}/include/vtk-${VTK_SHORT_VERSION}/vtkChemistryConfigure.h")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/vtk-${VTK_SHORT_VERSION}/vtkChemistryConfigure.h" "${SOURCE_PATH}" "not/existing" IGNORE_UNCHANGED)
endif()
# =============================================================================
# Usage
configure_file("${CMAKE_CURRENT_LIST_DIR}/usage" "${CURRENT_PACKAGES_DIR}/share/${PORT}/usage" COPYONLY)
# Handle copyright
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/Copyright.txt")

vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/share/vtk/VTK-vtk-module-properties.cmake" "_vtk_module_import_prefix}/lib/vtk-9.3/hierarchy" "_vtk_module_import_prefix}$<$<CONFIG:Debug>:/debug>/lib/vtk-9.3/hierarchy")
