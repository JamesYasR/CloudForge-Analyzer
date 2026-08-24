; =============================================================================
; CloudForgeAnalyzer NSIS 基础打包脚本
;
; 设计目标：
;   把已经完成运行时收集的 APP_BUILD_DIR（stage）递归打包为独立安装包。
;   CMake install 阶段已经完成：
;     - exe 与 vcpkg release DLL 全量复制
;     - windeployqt（Qt 插件 + Qt DLL + MSVC 运行库）
;   本脚本额外提供 VCPKG_BIN_DIR 编译期兜底收集，可独立重复执行。
;
; 命令行示例（PowerShell）：
;   makensis.exe /DAPP_VERSION=1.0.0 `
;     /DAPP_BUILD_DIR=D:\repo\stage `
;     /DVCPKG_BIN_DIR=D:\repo\vcpkg_installed\x64-windows\bin `
;     /DOUTPUT_DIR=D:\repo\artifacts `
;     cmake\nsis\CloudForgeAnalyzer.nsi
; =============================================================================

Unicode true
SetCompressor /SOLID lzma
RequestExecutionLevel admin
ManifestDPIAware true

; ----------------------------- 可覆盖参数 ---------------------------------
!ifndef APP_VERSION
  !define APP_VERSION "1.0.0"
!endif

!ifndef APP_BUILD_DIR
  !define APP_BUILD_DIR "..\..\build\stage"
!endif

!ifndef OUTPUT_DIR
  !define OUTPUT_DIR "..\..\artifacts"
!endif

!define APP_NAME "CloudForge Analyzer"
!define APP_EXE "CloudForgeAnalyzer.exe"
!define APP_REG_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\CloudForgeAnalyzer"

; --------------------- 编译期兜底：自动收集依赖 ---------------------------
; 若外部（CMake install）已经收集过，xcopy 会幂等覆盖，不影响结果。
!ifdef VCPKG_BIN_DIR
  !system 'cmd /C if not exist "${APP_BUILD_DIR}" mkdir "${APP_BUILD_DIR}"'
  !system 'cmd /C xcopy /E /I /Y "${VCPKG_BIN_DIR}\*.dll" "${APP_BUILD_DIR}" >nul'
!endif

; 可选：直接指定 windeployqt 时，在编译 NSIS 脚本阶段补全 Qt 运行库。
; 未定义 QT_WINDEPLOYQT 时，依赖 CMake install 中生成的 Qt 部署脚本完成。
!ifdef QT_WINDEPLOYQT
  !system 'cmd /C ""${QT_WINDEPLOYQT}" --release --compiler-runtime --no-translations --dir "${APP_BUILD_DIR}" "${APP_BUILD_DIR}\${APP_EXE}""'
!endif

; ------------------------------ 界面定义 -----------------------------------
!include "MUI2.nsh"

!define MUI_ABORTWARNING

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

!insertmacro MUI_LANGUAGE "SimpChinese"
!insertmacro MUI_LANGUAGE "English"

; ------------------------------ 基本信息 -----------------------------------
Name "${APP_NAME}"
OutFile "${OUTPUT_DIR}\CloudForgeAnalyzer-Setup-${APP_VERSION}.exe"
InstallDir "$PROGRAMFILES64\CloudForgeAnalyzer"
InstallDirRegKey HKLM "${APP_REG_KEY}" "InstallLocation"

VIProductVersion "${APP_VERSION}.0"
VIAddVersionKey "ProductName" "${APP_NAME}"
VIAddVersionKey "ProductVersion" "${APP_VERSION}"
VIAddVersionKey "FileVersion" "${APP_VERSION}.0"
VIAddVersionKey "FileDescription" "CloudForge Analyzer Setup"
VIAddVersionKey "LegalCopyright" "CloudForge Analyzer"

; ------------------------------ 安装区段 -----------------------------------
Section "CloudForge Analyzer" SEC_MAIN
  SectionIn RO

  SetOutPath "$INSTDIR"

  ; 递归收集 stage 下全部文件：主程序、DLL、Qt 插件、qt.conf、PCDfiles。
  ; 排除链接/调试中间产物，避免把开发文件装进用户机器。
  File /r /x "*.pdb" /x "*.ilk" /x "*.exp" /x "*.lib" "${APP_BUILD_DIR}\*.*"

  ; 生成卸载器
  WriteUninstaller "$INSTDIR\Uninstall.exe"

  ; 卸载注册表（64 位注册表视图）
  SetRegView 64
  WriteRegStr HKLM "${APP_REG_KEY}" "DisplayName" "${APP_NAME}"
  WriteRegStr HKLM "${APP_REG_KEY}" "DisplayVersion" "${APP_VERSION}"
  WriteRegStr HKLM "${APP_REG_KEY}" "Publisher" "CloudForge Analyzer"
  WriteRegStr HKLM "${APP_REG_KEY}" "DisplayIcon" "$INSTDIR\${APP_EXE}"
  WriteRegStr HKLM "${APP_REG_KEY}" "UninstallString" '"$INSTDIR\Uninstall.exe"'
  WriteRegStr HKLM "${APP_REG_KEY}" "InstallLocation" "$INSTDIR"
  WriteRegDWORD HKLM "${APP_REG_KEY}" "NoModify" 1
  WriteRegDWORD HKLM "${APP_REG_KEY}" "NoRepair" 1

  ; 开始菜单与桌面快捷方式（工作目录默认为 $INSTDIR，程序依赖相对路径 PCDfiles）
  CreateDirectory "$SMPROGRAMS\${APP_NAME}"
  CreateShortcut "$SMPROGRAMS\${APP_NAME}\${APP_NAME}.lnk" "$INSTDIR\${APP_EXE}" "" "$INSTDIR\${APP_EXE}" 0
  CreateShortcut "$DESKTOP\${APP_NAME}.lnk" "$INSTDIR\${APP_EXE}" "" "$INSTDIR\${APP_EXE}" 0
SectionEnd

; ------------------------------ 卸载区段 -----------------------------------
Section "Uninstall"
  SetRegView 64

  Delete "$DESKTOP\${APP_NAME}.lnk"
  Delete "$SMPROGRAMS\${APP_NAME}\${APP_NAME}.lnk"
  RMDir "$SMPROGRAMS\${APP_NAME}"

  Delete "$INSTDIR\Uninstall.exe"
  Delete "$INSTDIR\${APP_EXE}"
  ; 递归删除 stage 解包出的全部文件与目录
  RMDir /r "$INSTDIR"

  DeleteRegKey HKLM "${APP_REG_KEY}"
SectionEnd
