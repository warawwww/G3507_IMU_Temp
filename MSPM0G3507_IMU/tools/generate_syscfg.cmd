@echo off
setlocal

set "PROJECT_ROOT=%~dp0.."

if not defined SYSCONFIG_ROOT (
  echo [ERROR] SYSCONFIG_ROOT is not defined
  exit /b 1
)

if not defined MSPM0_SDK_ROOT (
  echo [ERROR] MSPM0_SDK_ROOT is not defined
  exit /b 1
)

call "%SYSCONFIG_ROOT%\sysconfig_cli.bat" ^
  --product "%MSPM0_SDK_ROOT%\.metadata\product.json" ^
  --script "%PROJECT_ROOT%\cmsis_dsp_empty.syscfg" ^
  --output "%PROJECT_ROOT%\generated\syscfg" ^
  --compiler gcc

if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

powershell -NoProfile -ExecutionPolicy Bypass -File "%PROJECT_ROOT%\tools\reserve_imu_flash.ps1" "%PROJECT_ROOT%\generated\syscfg\device_linker.lds"

exit /b %ERRORLEVEL%
