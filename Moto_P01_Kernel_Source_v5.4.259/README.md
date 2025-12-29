# Moto P01 Kernel Source

## Overview
This package contains the Linux kernel source code and GPL-licensed drivers for the Moto P01 (A113L2 platform).

**Kernel Version:** 5.4.259
**Platform:** Amlogic A113L2

## Contents
1. `kernel/`: The Linux Kernel source code (GPLv2).
2. `hardware/aml-5.4/wifi/`: WiFi driver source code (GPLv2).
3. `prebuilts/`: GCC Toolchain (included for offline build convenience).
4. `Kernel_build.sh`: Helper script to build the kernel image.

## Exclusions (Not included)
- U-Boot (Bootloader): Provided separately or via upstream.
- User-space libraries (e.g., BlueZ, proprietary vendor HALs).
- Proprietary Firmware binaries.

## How to Build
1. Run `./Kernel_build.sh`
2. Output will be located in `out/`

## Legal Notice
This software is provided "AS IS", without warranty of any kind.
Source code is released in compliance with GPLv2 obligations.
