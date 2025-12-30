================================================================================
              Amlogic 5.4.259 Original Kernel Source Code Statement
================================================================================

I. Kernel Source
----------------
The kernel/ directory contains the original kernel source code provided by Amlogic:
- Version: Linux 5.4.259
- Chip Platform: Amlogic A64 SmartHome Platform
- Official Version: Factory unmodified version

II. Code Status
---------------
We solemnly declare:
1. This kernel source code comes from Amlogic official sources, without any functional modifications
2. The integrity and original state of the factory code has been maintained
3. Published solely to comply with GPL open source licensing requirements

III. Compilation Information
----------------------------
1. Compilation toolchain: gcc-10.2 (prebuilts/gcc/gcc-10.2/bin)
2. Configuration file: meson64_a64_smarthome_defconfig
3. Compilation script: Kernel_build.sh

IV. File Description
--------------------
├── kernel/              # Amlogic original kernel source code (unmodified)
├── hardware/            # WiFi-related code
├── prebuilts/           # Pre-built toolchain
├── build.log           # Compilation log example
├── Kernel_build.sh     # Compilation script
├── LICENSE            # Open source license
├── README.md          # Project description
└── ORIGINAL_KERNEL_README.txt  # This file

V. Usage Instructions
---------------------
1. Compile the kernel:
   ./Kernel_build.sh

2. Compilation output:
   - Kernel image: out/arch/arm64/boot/Image.gz
   - Device trees: out/arch/arm64/boot/dts/amlogic/*.dtb

VI. License
-----------
This kernel code follows the GPL v2 license. Detailed terms can be found in the LICENSE file.

- Release Date: December 29, 2025

================================================================================
                    "Keep Original, Open Source Compliant"
================================================================================
