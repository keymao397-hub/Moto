# Moto P01 Kernel Source

## 1. Download
*   **Offline Kit**: Download from [Releases](../../releases) (Includes Toolchain).
*   **Git Clone**: `git clone https://github.com/keymao397-hub/Moto.git`

## 2. Build Instructions
If you cloned the repo, install GCC first:
```bash
sudo apt install gcc-aarch64-linux-gnu make
```

**Run Build:**
```bash
chmod +x Kernel_build.sh
./Kernel_build.sh
```

**Output:** `out/arch/arm64/boot/Image.gz`
