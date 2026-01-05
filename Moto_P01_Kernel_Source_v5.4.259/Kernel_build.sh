#!/bin/bash
# Kernel Build Script for Moto P01 (5.4.259)

# 获取当前目录绝对路径
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

SOURCE_DIR="$ROOT_DIR/kernel"
OUT_DIR="$ROOT_DIR/out"
# 日志文件路径
LOG_FILE="$ROOT_DIR/build.log"

# 如果有 WiFi 源码，导出路径供 Makefile 使用 (视具体编译逻辑而定)
export WIFI_SRC_DIR="$ROOT_DIR/hardware/aml-5.4/wifi"

# 设置工具链路径
if [ -d "$ROOT_DIR/prebuilts/gcc/gcc-10.2/bin" ]; then
    export PATH="$ROOT_DIR/prebuilts/gcc/gcc-10.2/bin:$PATH"
    echo "Using included Toolchain." | tee -a "$LOG_FILE"
else
    echo "Using system GCC." | tee -a "$LOG_FILE"
fi

export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=aarch64-none-linux-gnu-

# 清空旧的日志文件
: > "$LOG_FILE"

echo "========================================" | tee -a "$LOG_FILE"
echo " Building Kernel: 5.4.259" | tee -a "$LOG_FILE"
echo " Config: meson64_a64_smarthome_defconfig" | tee -a "$LOG_FILE"
echo " Log file: $LOG_FILE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

mkdir -p "$OUT_DIR"

echo "==> Configuring..." | tee -a "$LOG_FILE"
make -C "$SOURCE_DIR" O="$OUT_DIR" meson64_a64_smarthome_defconfig 2>&1 | tee -a "$LOG_FILE"

# 检查 make 命令的返回状态（不是 tee 命令的）
if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "Error: Defconfig failed!" | tee -a "$LOG_FILE"
    exit 1
fi

echo "==> Building Image & Modules..." | tee -a "$LOG_FILE"
make -C "$SOURCE_DIR" O="$OUT_DIR" -j$(nproc) Image.gz dtbs modules 2>&1 | tee -a "$LOG_FILE"

if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    echo "Build SUCCESS!" | tee -a "$LOG_FILE"
    echo "Output directory: $OUT_DIR" | tee -a "$LOG_FILE"
    echo "Kernel image: $OUT_DIR/arch/arm64/boot/Image.gz" | tee -a "$LOG_FILE"
    echo "Build log saved to: $LOG_FILE" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
else
    echo "" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    echo "Build FAILED!" | tee -a "$LOG_FILE"
    echo "Check the log file for details: $LOG_FILE" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    exit 1
fi
