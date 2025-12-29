# Moto P01 (A113L2) Kernel Source & Offline Build Kit

这是一个包含完整开发环境的内核源码包。
This is an "All-in-One" offline build package including Source Code, GCC Toolchain, and Build Scripts.

## 📥 下载 (Download)
**请前往 [Releases 页面](../../releases) 下载最新的 `.tar.gz` 包。**

Please go to the [Releases Page](../../releases) to download the full source package.

## 📦 包内包含 (Contents)
- **Kernel 5.4.259** Source Code
- **GCC 10.2** Toolchain (Prebuilt)
- **Automated Build Scripts** (No env setup required)

## 🚀 如何使用 (How to use)

1. Download `Moto_P01_Kernel_Offline_Kit_v1.0.tar.gz` from Releases.
2. Extract the package:
   ```bash
   tar -xzvf Moto_P01_Kernel_Offline_Kit_v1.0.tar.gz
   cd Moto_P01_Package
   ```
3. Run the build script:
   ```bash
   ./Kernel_build.sh
   ```
4. The output image will be in `out/arch/arm64/boot/`.
