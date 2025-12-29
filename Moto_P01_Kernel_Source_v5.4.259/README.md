# Moto P01 (A113L2) Linux Kernel Source

[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](LICENSE)
[![Platform: Amlogic](https://img.shields.io/badge/Platform-Amlogic%20A113L2-green.svg)](http://www.amlogic.com)

## 📖 Overview

This repository contains the **Linux Kernel 5.4** source code and GPL-compliant drivers for the **Moto P01** device, powered by the **Amlogic A113L2** SoC.

It allows developers and enthusiasts to build the kernel image and WiFi modules from source.

- **Kernel Version**: 5.4.259
- **Architecture**: ARM64 (AArch64)
- **Defconfig**: `meson64_a64_smarthome_defconfig`

---

## 📂 Repository Contents

| Directory | Description |
| :--- | :--- |
| `kernel/` | The core Linux Kernel source code. |
| `hardware/` | Amlogic WiFi drivers (W1/W1u), linked as kernel modules. |
| `Kernel_build.sh` | Automated helper script to compile the kernel. |
| `out/` | Build artifacts (generated after compilation). |

> **Note:** This repository follows the standard Linux GPL requirements. Proprietary user-space libraries, bootloaders (U-Boot), and firmware blobs are **not** included.

---

## 📥 How to Download

You have two options to obtain the source code:

### ✅ Option A: Offline Build Kit (Recommended)
If you want to build immediately without installing dependencies, download the **Release Tarball** from the **[Releases Page](../../releases)**.
*   **Includes prebuilt GCC Toolchain.**
*   No extra environment setup required.

### 💻 Option B: Git Clone (For Developers)
Clone the repository directly:
```bash
git clone https://github.com/keymao397-hub/Moto.git
cd Moto
