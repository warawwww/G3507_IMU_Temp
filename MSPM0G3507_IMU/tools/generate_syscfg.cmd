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

exit /b %ERRORLEVEL%