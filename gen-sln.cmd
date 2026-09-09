@echo off
rem ============================================================
rem  Generate the Visual Studio solution for CloudForgeAnalyzer.
rem  Usage: double-click this file (Windows only).
rem  Requires (one-time, already done on the dev machine):
rem    - Visual Studio 2022 with C++ tools
rem    - user env vars PCL_ROOT / QTDIR  (prebuilt deps at D:\Dev)
rem  Output: build\sln\CloudForgeAnalyzer.sln
rem
rem  NOTE: intentionally written with goto instead of if (...) blocks.
rem  %ProgramFiles(x86)% expands to a path containing parentheses,
rem  which breaks cmd's parser inside parenthesized blocks.
rem ============================================================
setlocal

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" goto vswhere_ok
echo [ERROR] vswhere not found: %VSWHERE%
pause
exit /b 1
:vswhere_ok

set "VSDIR="
rem cd into the Installer dir first: a quoted path containing "(x86)" inside
rem for /f's in (...) clause truncates at the ")" and breaks the parser.
pushd "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
for /f "usebackq delims=" %%i in (`.\vswhere.exe -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
popd
if defined VSDIR goto vsdir_ok
echo [ERROR] Visual Studio with C++ workload not found.
pause
exit /b 1
:vsdir_ok

set "CMAKE=%VSDIR%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if exist "%CMAKE%" goto cmake_ok
echo [ERROR] cmake not found: %CMAKE%
pause
exit /b 1
:cmake_ok

if defined PCL_ROOT goto pcl_ok
echo [ERROR] env var PCL_ROOT not set - see docs for one-time setup.
pause
exit /b 1
:pcl_ok

echo Using CMake: %CMAKE%
echo.
"%CMAKE%" -S "%~dp0." -B "%~dp0build\sln" -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="%QTDIR%;%PCL_ROOT%;%PCL_ROOT%/3rdParty/Eigen3"
echo.
echo Done. Open build\sln\CloudForgeAnalyzer.sln in Visual Studio,
echo set CloudForgeAnalyzer as startup project, then press F5.
pause
