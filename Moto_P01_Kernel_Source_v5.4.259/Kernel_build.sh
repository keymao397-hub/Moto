#!/bin/bash
# Kernel Build Script for Moto P01 (5.4.259)

# 获取当前目录绝对路径
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

SOURCE_DIR="$ROOT_DIR/kernel"
OUT_DIR="$ROOT_DIR/out"
# 如果有 WiFi 源码，导出路径供 Makefile 使用 (视具体编译逻辑而定)
export WIFI_SRC_DIR="$ROOT_DIR/hardware/aml-5.4/wifi"

# 设置工具链路径
if [ -d "$ROOT_DIR/prebuilts/gcc/gcc-10.2/bin" ]; then
    export PATH="$ROOT_DIR/prebuilts/gcc/gcc-10.2/bin:$PATH"
    echo "Using included Toolchain."
else
    echo "Using system GCC."
fi

export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=aarch64-none-linux-gnu-

echo "========================================"
echo " Building Kernel: 5.4.259"
echo " Config: meson64_a64_smarthome_defconfig"
echo "========================================"

mkdir -p "$OUT_DIR"

echo "==> Configuring..."
make -C "$SOURCE_DIR" O="$OUT_DIR" meson64_a64_smarthome_defconfig

if [ $? -ne 0 ]; then
    echo "Error: Defconfig failed!"
    exit 1
fi

echo "==> Building Image & Modules..."
make -C "$SOURCE_DIR" O="$OUT_DIR" -j$(nproc) Image.gz dtbs modules

if [ $? -eq 0 ]; then
    echo "Build SUCCESS!"
    echo "Output: $OUT_DIR/arch/arm64/boot/Image.gz"
else
    echo "Build FAILED!"
    exit 1
fi
