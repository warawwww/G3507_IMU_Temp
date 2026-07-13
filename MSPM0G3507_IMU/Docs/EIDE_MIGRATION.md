# MSPM0G3507 工程迁移到 EIDE 记录

本文记录已经验证成功的开发链：

```text
独立 SysConfig GUI 编辑 .syscfg
        ↓
EIDE Pre-build 调用 SysConfig CLI
        ↓
生成初始化代码和链接文件
        ↓
EIDE + arm-none-eabi-gcc 编译、链接
        ↓
OpenOCD + XDS110 烧录
```

## 0. 当前基线

| 项目 | 当前配置 |
| --- | --- |
| MCU | MSPM0G3507，Arm Cortex-M0+ |
| 开发板 | LP-MSPM0G3507 |
| MSPM0 SDK | `2.10.0.04` |
| SysConfig | `1.28.0+4712` |
| EIDE GCC | Arm GNU Toolchain `14.3.1` |
| SysConfig 文件 | `cmsis_dsp_empty.syscfg` |
| 调试器 | LaunchPad 板载 XDS110 |
| EIDE 输出目录 | `build/Debug` |
| SysConfig 输出目录 | `generated/syscfg` |

当前进度：

- [x] 独立 SysConfig GUI 编辑和保存
- [x] SysConfig CLI 生成 GCC 文件
- [x] SysConfig CLI 接入 EIDE Pre-build
- [x] EIDE 编译和链接
- [x] 生成 ELF 和 MAP
- [x] 安装新版 OpenOCD
- [x] OpenOCD 连接 XDS110
- [x] EIDE 烧录
- [ ] Cortex-Debug 调试

## 1. 配置独立 SysConfig GUI

### 1.1 安装工具

安装以下软件：

```text
SysConfig 1.28.0
MSPM0 SDK 2.10.0.04
```

本机路径：

```text
SysConfig：D:/Environment/TI_sysconfig
MSPM0 SDK：D:/diansai/ti/mspm0_sdk_2_10_00_04
```

参考资料：

- [TI SysConfig 产品页](https://www.ti.com/tool/SYSCONFIG)
- [MSPM0 SysConfig 指南](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/2_10_00_04/docs/english/tools/sysconfig_guide/doc_guide/doc_guide-srcs/sysconfig_guide.html)

### 1.2 打开项目配置

在 PowerShell 中执行：

```powershell
& 'D:\Environment\TI_sysconfig\sysconfig_gui.bat' `
  --product 'D:\diansai\ti\mspm0_sdk_2_10_00_04\.metadata\product.json' `
  'E:\one_drive\OneDrive - CUHK-Shenzhen\diansai\G3507_IMU_Temp\MSPM0G3507_IMU\cmsis_dsp_empty.syscfg'
```

`--product` 指定 MSPM0 SDK。最后一个参数指定需要编辑的 `.syscfg`。

### 1.3 检查配置

在 GUI 中确认：

```text
Device：MSPM0G3507
Package：LQFP-64(PM)
Compiler：GCC
Board：LP-MSPM0G3507
```

Project Configuration 中启用：

```text
Linker File Generation
Startup File Reference
Option File Generation
Linker Libraries File Generation
DriverLib
CMSIS/DSP
```

Generated Files 中显示：

```text
device_linker.lds
device.lds.genlibs
device.opt
ti_msp_dl_config.c
ti_msp_dl_config.h
```

按 `Ctrl+S` 保存 `cmsis_dsp_empty.syscfg`。

### 1.4 命令行生成代码

在项目根目录执行：

```powershell
& 'D:\Environment\TI_sysconfig\sysconfig_cli.bat' `
  --product 'D:\diansai\ti\mspm0_sdk_2_10_00_04\.metadata\product.json' `
  --script '.\cmsis_dsp_empty.syscfg' `
  --output '.\generated\syscfg' `
  --compiler gcc
```

输出目录包含：

```text
generated/syscfg/
├── device.lds.genlibs
├── device.opt
├── device_linker.lds
├── ti_msp_dl_config.c
└── ti_msp_dl_config.h
```

`.gitignore` 中加入：

```gitignore
/generated/
```

`.syscfg` 是配置源文件，`generated/syscfg` 在构建时自动刷新。

## 2. 把 SysConfig 和 MSPM0 SDK 接入 EIDE

### 2.1 项目目录

相关文件放置如下：

```text
MSPM0G3507_IMU/
├── cmsis_dsp_empty.syscfg
├── User/
│   ├── main.c
│   ├── app.c
│   └── app.h
├── BSP/
│   ├── bsp.c
│   └── bsp.h
├── Drivers/
│   └── README.md
├── Algorithm/
│   └── README.md
├── generated/
│   └── syscfg/
├── startup/
│   └── startup_mspm0g350x_gcc.c
├── tools/
│   └── generate_syscfg.cmd
└── .eide/
    ├── eide.yml
    └── env.ini
```

`startup_mspm0g350x_gcc.c` 来源：

```text
MSPM0 SDK 2.10.0.04/
source/ti/devices/msp/m0p/startup_system_files/gcc/
startup_mspm0g350x_gcc.c
```

### 2.2 配置 EIDE 环境变量

打开 `.eide/env.ini`，写入：

```ini
## Global Variables

MSPM0_SDK_ROOT=D:/diansai/ti/mspm0_sdk_2_10_00_04
SYSCONFIG_ROOT=D:/Environment/TI_sysconfig

[debug]
MCU_RAM_SIZE=0x8000
MCU_ROM_SIZE=0x20000
```

变量语法：

```text
EIDE 配置：${MSPM0_SDK_ROOT}
Windows CMD：%MSPM0_SDK_ROOT%
PowerShell：$env:MSPM0_SDK_ROOT
```

### 2.3 创建 SysConfig 生成脚本

`tools/generate_syscfg.cmd` 内容：

```bat
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
```

### 2.4 添加 EIDE Pre-build

进入：

```text
EIDE → 构建器选项 → 用户任务
```

添加 Pre-build：

```text
名称：Syscfg Pre-build
命令："${ProjectRoot}\tools\generate_syscfg.cmd"
失败后停止构建：启用
```

构建时首先执行 SysConfig，随后执行 GCC 编译。

### 2.5 添加源文件

在 EIDE 项目资源中加入：

```text
Algorithm
User
BSP
Drivers
startup/startup_mspm0g350x_gcc.c
generated/syscfg
```

当前 EIDE 编译以下 C 文件：

```text
User/main.c
User/app.c
BSP/bsp.c
generated/syscfg/ti_msp_dl_config.c
startup/startup_mspm0g350x_gcc.c
```

### 2.6 添加 Include 路径

进入：

```text
EIDE → C/C++ 属性 → 包含目录
```

添加：

```text
Algorithm
BSP
Drivers
User
generated/syscfg
${MSPM0_SDK_ROOT}/source
${MSPM0_SDK_ROOT}/source/third_party/CMSIS/Core/Include
${MSPM0_SDK_ROOT}/source/third_party/CMSIS/DSP/Include
```

这些路径分别提供：

```text
SysConfig 生成头文件
MSPM0G3507 芯片头文件和 DriverLib
CMSIS Core
CMSIS-DSP
```

### 2.7 添加宏定义

进入：

```text
EIDE → C/C++ 属性 → 预处理器定义
```

添加：

```text
__MSPM0G3507__
__USE_SYSCONFIG__
ARM_MATH_CM0
```

### 2.8 配置 CPU 和主链接脚本

进入 EIDE 构建配置，设置：

```text
CPU：Cortex-M0+
Hardware Floating Point：none
Thumb Mode：thumb
Linker Script：generated/syscfg/device_linker.lds
```

`device_linker.lds` 定义 MSPM0G3507 的 Flash、SRAM、向量表、栈和数据段。

### 2.9 配置库目录

进入：

```text
EIDE → C/C++ 属性 → 库搜索目录
```

添加：

```text
${MSPM0_SDK_ROOT}/source
```

`device.lds.genlibs` 使用这个根目录查找：

```text
third_party/CMSIS/DSP/lib/gcc/m0p/arm_cortexM0l_math.a
ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a
```

### 2.10 配置链接器

进入：

```text
EIDE → 构建器选项 → 链接器
```

链接器附加选项：

```text
-nostartfiles -static
```

链接库选项：

```text
-l:third_party/CMSIS/DSP/lib/gcc/m0p/arm_cortexM0l_math.a -Tgenerated/syscfg/device.lds.genlibs -lm
```

作用：

```text
-nostartfiles    使用项目内的 TI startup
-static          静态链接
-l:...math.a     在 C 标准库之前显式引入 CMSIS-DSP，满足 PID 初始化的 memset 依赖
-T...genlibs     读取 SysConfig 生成的 DriverLib/CMSIS-DSP 库清单
-lm              链接标准数学库
```

`device.lds.genlibs` 放在链接库选项中，使静态库在目标文件之后参与链接。CMSIS-DSP 库同时显式列出，用于规避 GCC 处理 `-T...genlibs` 时的静态库扫描顺序问题。

### 2.11 构建

点击 EIDE 普通 Build。

构建日志顺序：

```text
pre-build tasks
Syscfg Pre-build
编译项目 C 文件
链接 MSPM0G3507_IMU.elf
打印内存使用量
```

成功输出：

```text
build/Debug/MSPM0G3507_IMU.elf
build/Debug/MSPM0G3507_IMU.map
build/Debug/compile_commands.json
build/Debug/builder.params
```

2026-07-13 实测结果：

```text
FLASH：496 B / 128 KiB
SRAM：0 B / 32 KiB
EIDE GCC：14.3.1
```

第二阶段完成。

## 3. 新版 OpenOCD + XDS110 烧录

### 3.1 安装新版 OpenOCD

在 VS Code 中安装 TI Embedded Debug 扩展，然后通过扩展的 `Install Dependencies` 下载最新版 OpenOCD。

将下载得到的整个 `openocd` 文件夹放到固定位置。本机使用：

```text
D:/diansai/ti/openocd/
├── bin/openocd.exe
└── share/openocd/scripts/
```

这个版本已经包含 XDS110、MSPM0 target 和 MSPM0 Flash 驱动所需的配置。

### 3.2 修改 EIDE 的 OpenOCD 路径

打开 VS Code 设置，搜索：

```text
EIDE OpenOCD ExePath
```

将路径设置为：

```text
D:\diansai\ti\openocd\bin\openocd.exe
```

对应的 `settings.json` 内容为：

```json
"EIDE.OpenOCD.ExePath": "D:\\diansai\\ti\\openocd\\bin\\openocd.exe"
```

可以在 PowerShell 中检查版本：

```powershell
& 'D:\diansai\ti\openocd\bin\openocd.exe' --version
```

### 3.3 配置 EIDE 烧录器

进入 EIDE 的烧录配置，选择 `OpenOCD`，设置：

```text
Interface：xds110
Target：ti_mspm0
Base Address：0x00000000
```

本机 OpenOCD 使用的脚本为：

```text
share/openocd/scripts/interface/xds110.cfg
share/openocd/scripts/target/ti_mspm0.cfg
share/openocd/scripts/board/ti_mspm0_launchpad.cfg
```

### 3.4 烧录并检查结果

连接 LaunchPad，先执行 EIDE Build，再执行 EIDE Flash。

出现以下日志表示写入和校验成功：

```text
** Programming Finished **
** Verify Started **
** Verified OK **
XDS110: disconnected
```

`checksum mismatch - attempting binary compare` 表示 OpenOCD 从快速校验切换到逐字节比较。最终出现 `Verified OK` 就表示烧录成功。

第三阶段完成。

## 4. 故障定位

| 现象 | 检查内容 |
| --- | --- |
| GUI 报 `/ti/driverlib/...` 不存在 | GUI 启动命令中的 SDK `product.json` |
| Pre-build 报变量未定义 | `.eide/env.ini` 中的 `MSPM0_SDK_ROOT` 和 `SYSCONFIG_ROOT` |
| 编译找不到 `arm_math.h` | CMSIS-DSP Include 路径 |
| 编译找不到 `ti/driverlib/...` | `${MSPM0_SDK_ROOT}/source` Include 路径 |
| 链接找不到 `DL_Common_delayCycles` | `device.lds.genlibs` 位于链接库选项中 |
| `cannot open linker script ... Invalid argument` | `device.lds.genlibs` 使用项目相对路径 |
| OpenOCD 找不到 MSPM0 配置 | 使用 TI Embedded Debug 下载的新版 OpenOCD，并检查 `share/openocd/scripts` |
| OpenOCD 找不到 XDS110 | XDS110 Debug Probe 的 WinUSB 驱动 |

## 5. 验证记录

| 日期 | 项目 | 结果 |
| --- | --- | --- |
| 2026-07-12 | SysConfig GUI | `1.28.0+4712` 成功加载 MSPM0 SDK |
| 2026-07-12 | SysConfig CLI | 成功生成五个 GCC 文件 |
| 2026-07-13 | EIDE Pre-build | 成功自动调用 SysConfig CLI |
| 2026-07-13 | EIDE Build | GCC 14.3.1 编译链接成功，生成 ELF 和 MAP |
| 2026-07-13 | XDS110 Windows 驱动 | TI `2.0.0.2` / WinUSB |
| 2026-07-13 | OpenOCD | 成功通过 XDS110 连接 MSPM0G3507 |
| 2026-07-13 | EIDE Flash | `Programming Finished`，`Verified OK` |
