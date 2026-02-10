#!/bin/bash
# ================================================================
# Build minimal Linux + OpenSBI for Custom RV32IMA SoC
# Run inside Docker: docker/Dockerfile (riscv32-linux-musl toolchain)
#
# Usage:
#   docker build -t riscv-toolchain docker/
#   docker run -it --rm \
#     -v $(pwd):/output \
#     -v riscv-build:/work \
#     riscv-toolchain bash /output/scripts/build_all.sh
# ================================================================
set -e

CROSS=riscv32-linux-musl-
WORKDIR=/work
OUTPUTDIR=/output
NPROC=$(nproc)

echo "=============================================="
echo "  Building Minimal Linux for RV32IMA SoC"
echo "  Toolchain: ${CROSS}gcc"
echo "  Threads: ${NPROC}"
echo "=============================================="

# ================================================================
# Step 1: Build Busybox (static)
# ================================================================
echo ""
echo "=== Step 1: Building Busybox ==="

cd $WORKDIR

if [ ! -d "busybox" ]; then
    echo "Downloading busybox..."
    git clone --depth 1 --branch 1_36_stable https://git.busybox.net/busybox
fi

cd busybox
make distclean || true
make ARCH=riscv CROSS_COMPILE=$CROSS defconfig

# Enable static, disable problematic features
sed -i 's/# CONFIG_STATIC is not set/CONFIG_STATIC=y/' .config
sed -i 's/CONFIG_HWCLOCK=y/# CONFIG_HWCLOCK is not set/' .config
sed -i 's/CONFIG_FEATURE_HWCLOCK_ADJTIME_FHS=y/# CONFIG_FEATURE_HWCLOCK_ADJTIME_FHS is not set/' .config
sed -i 's/CONFIG_FEATURE_HWCLOCK_LONG_OPTIONS=y/# CONFIG_FEATURE_HWCLOCK_LONG_OPTIONS is not set/' .config
sed -i 's/CONFIG_BC=y/# CONFIG_BC is not set/' .config
sed -i 's/CONFIG_DC=y/# CONFIG_DC is not set/' .config
make ARCH=riscv CROSS_COMPILE=$CROSS oldconfig < /dev/null
make -j${NPROC} ARCH=riscv CROSS_COMPILE=$CROSS

echo "Busybox built: $(file busybox)"

# ================================================================
# Step 2: Create initramfs
# ================================================================
echo ""
echo "=== Step 2: Creating initramfs ==="

cd $WORKDIR
rm -rf initramfs
mkdir -p initramfs/{bin,sbin,etc,proc,sys,dev,tmp,root}

cp busybox/busybox initramfs/bin/
chmod +x initramfs/bin/busybox

# Create symlinks
cd initramfs/bin
for cmd in sh ash ls cat echo mkdir mount umount ps kill sleep clear pwd \
           cp mv rm ln grep head tail wc sort uniq tr sed awk \
           dmesg free top df du date id whoami hostname uname; do
    ln -sf busybox $cmd 2>/dev/null || true
done
cd ../sbin
for cmd in init halt reboot poweroff; do
    ln -sf ../bin/busybox $cmd
done

cd $WORKDIR/initramfs

# Create init script
cat > init << 'INITEOF'
#!/bin/sh

# Mount essential filesystems
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || true

# Create device nodes (fallback if devtmpfs not available)
mknod -m 622 /dev/console c 5 1 2>/dev/null || true
mknod -m 666 /dev/null c 1 3 2>/dev/null || true
mknod -m 666 /dev/zero c 1 5 2>/dev/null || true
mknod -m 666 /dev/ttyS0 c 4 64 2>/dev/null || true

# Print banner
echo ""
echo "=================================="
echo "  RV32IMA Linux - Shell Ready!"
echo "=================================="
echo ""
echo "Type 'help' for available commands"
echo ""

# Start shell on console
exec /bin/sh
INITEOF
chmod +x init

# Create /etc files
echo "root:x:0:0:root:/root:/bin/sh" > etc/passwd
echo "root:x:0:" > etc/group
echo "soc" > etc/hostname

# Create initramfs cpio
cd $WORKDIR/initramfs
find . | cpio -o -H newc > ../initramfs.cpio 2>/dev/null
echo "Initramfs size: $(du -sh ../initramfs.cpio | cut -f1)"

# ================================================================
# Step 3: Build Linux kernel (minimal config)
# ================================================================
echo ""
echo "=== Step 3: Building Linux kernel ==="

cd $WORKDIR

# Use local kernel source if mounted, otherwise clone
if [ -d "/output/linux-6.1.25" ]; then
    echo "Using local kernel source: linux-6.1.25"
    if [ ! -d "linux" ] || [ ! -f "linux/.local_copy" ]; then
        rm -rf linux
        echo "Copying kernel source (tar pipe for speed)..."
        cd /output/linux-6.1.25 && tar cf - --exclude='.git' --exclude='*.o' --exclude='*.ko' --exclude='*.cmd' . | (mkdir -p $WORKDIR/linux && cd $WORKDIR/linux && tar xf -)
        cd $WORKDIR
        touch linux/.local_copy
    fi
else
    if [ ! -d "linux" ]; then
        echo "Cloning Linux kernel 6.1.y..."
        git clone --depth 1 --branch linux-6.1.y \
            https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git
    fi
fi

cd linux

# Clean stale objects from any previous config (e.g. SMP=y -> SMP=n)
make ARCH=riscv CROSS_COMPILE=$CROSS mrproper

# Start from rv32_defconfig (provides minimum RISC-V support)
make ARCH=riscv CROSS_COMPILE=$CROSS rv32_defconfig

# Disable M-mode before merging config (rv32_defconfig sets M_MODE=y)
sed -i 's/CONFIG_RISCV_M_MODE=y/# CONFIG_RISCV_M_MODE is not set/' .config

# Apply our minimal overrides
if [ -f "/output/scripts/soc_minimal.config" ]; then
    echo "Applying minimal SoC config..."
    # Use merge_config.sh for proper override (handles duplicates correctly)
    ARCH=riscv CROSS_COMPILE=$CROSS scripts/kconfig/merge_config.sh -m .config /output/scripts/soc_minimal.config
    make ARCH=riscv CROSS_COMPILE=$CROSS olddefconfig
fi

# Embed initramfs
sed -i '/CONFIG_INITRAMFS_SOURCE/d' .config
echo 'CONFIG_INITRAMFS_SOURCE="/work/initramfs.cpio"' >> .config
make ARCH=riscv CROSS_COMPILE=$CROSS olddefconfig

# Show final config summary
echo ""
echo "--- Config Summary ---"
echo "SMP:        $(grep CONFIG_SMP .config | head -1)"
echo "M_MODE:     $(grep CONFIG_RISCV_M_MODE .config | head -1 || echo 'not set')"
echo "SBI:        $(grep CONFIG_RISCV_SBI= .config | head -1 || echo 'not set')"
echo "NET:        $(grep CONFIG_NET= .config | head -1 || echo 'not set')"
echo "PCI:        $(grep CONFIG_PCI= .config | head -1 || echo 'not set')"
echo "EARLYCON:   $(grep CONFIG_SERIAL_EARLYCON .config | head -1)"
echo "INITRAMFS:  $(grep CONFIG_INITRAMFS_SOURCE .config | head -1)"
echo "BLOCK:      $(grep CONFIG_BLOCK .config | head -1 || echo 'not set')"
echo "MODULES:    $(grep CONFIG_MODULES .config | head -1 || echo 'not set')"
echo "HVC_SBI:    $(grep CONFIG_HVC_RISCV_SBI .config | head -1 || echo 'not set')"
echo "8250_OF:    $(grep CONFIG_SERIAL_OF_PLATFORM .config | head -1 || echo 'not set')"
echo "---"
echo ""

# Verify critical config
if grep -q "CONFIG_SMP=y" .config; then
    echo "ERROR: CONFIG_SMP=y still set! Spinlocks will deadlock on uniprocessor."
    exit 1
fi
if grep -q "CONFIG_RISCV_M_MODE=y" .config; then
    echo "ERROR: CONFIG_RISCV_M_MODE=y still set! Kernel must boot in S-mode via OpenSBI."
    exit 1
fi

# Build
make -j${NPROC} ARCH=riscv CROSS_COMPILE=$CROSS

# Copy artifacts
cp System.map /output/linux-6.1.25/System.map.new 2>/dev/null || true
cp vmlinux /output/linux-6.1.25/vmlinux.new 2>/dev/null || true
cp .config /output/linux-6.1.25/.config.minimal 2>/dev/null || true

echo "Kernel built!"
echo "  vmlinux: $(ls -lh vmlinux | awk '{print $5}')"
echo "  Image:   $(ls -lh arch/riscv/boot/Image | awk '{print $5}')"

# arch/riscv/boot/Image is already a raw binary, just copy it
cp arch/riscv/boot/Image ${WORKDIR}/linux_minimal.bin

# Verify initramfs is embedded (CPIO "070701" magic)
if grep -c '070701' ${WORKDIR}/linux_minimal.bin > /dev/null 2>&1; then
    echo "OK: initramfs CPIO archive found in kernel Image"
else
    echo "ERROR: No CPIO archive found in kernel Image!"
    echo "  Check that /work/initramfs.cpio exists and CONFIG_INITRAMFS_SOURCE is set."
    ls -la /work/initramfs.cpio 2>/dev/null || echo "  /work/initramfs.cpio MISSING"
    grep CONFIG_INITRAMFS_SOURCE .config || echo "  CONFIG_INITRAMFS_SOURCE not in .config"
    exit 1
fi

# ================================================================
# Step 4: Build OpenSBI fw_payload
# ================================================================
echo ""
echo "=== Step 4: Building OpenSBI fw_payload ==="

cd $WORKDIR

if [ ! -d "opensbi" ]; then
    echo "Cloning OpenSBI..."
    git clone --depth 1 --branch v1.4 https://github.com/riscv-software-src/opensbi.git
fi

cd opensbi

make distclean || true
make -j${NPROC} \
    CROSS_COMPILE=$CROSS \
    PLATFORM=generic \
    PLATFORM_RISCV_XLEN=32 \
    PLATFORM_RISCV_ISA=rv32imac \
    FW_PAYLOAD=y \
    FW_PAYLOAD_PATH=${WORKDIR}/linux_minimal.bin \
    FW_PAYLOAD_OFFSET=0x400000 \
    FW_PAYLOAD_FDT_ADDR=0x82200000

echo "OpenSBI built!"
echo "  fw_payload: $(ls -lh build/platform/generic/firmware/fw_payload.bin | awk '{print $5}')"

# ================================================================
# Step 5: Convert to hex
# ================================================================
echo ""
echo "=== Step 5: Converting to hex ==="

cd $WORKDIR
cp opensbi/build/platform/generic/firmware/fw_payload.bin fw_payload_minimal.bin

# Convert to little-endian hex (32-bit words)
python3 -c "
data = open('fw_payload_minimal.bin', 'rb').read()
while len(data) % 4:
    data += b'\x00'
with open('fw_payload_minimal.hex', 'w') as f:
    for i in range(0, len(data), 4):
        word = int.from_bytes(data[i:i+4], 'little')
        f.write(f'{word:08x}\n')
print(f'Converted {len(data)//4} words')
"

echo "Hex file: $(wc -l < fw_payload_minimal.hex) words"
echo "Size: $(du -sh fw_payload_minimal.hex | cut -f1)"

# Copy to output - directly replace fw_payload_earlycon.hex
cp fw_payload_minimal.hex /output/dts/fw_payload_minimal.hex
if [ -f /output/dts/fw_payload_earlycon.hex ]; then
    cp /output/dts/fw_payload_earlycon.hex /output/dts/fw_payload_earlycon.hex.bak
fi
cp fw_payload_minimal.hex /output/dts/fw_payload_earlycon.hex

# Final verification: check hex file for CPIO magic (LE 32-bit: "0707" -> 37303730)
if grep -q "37303730" /output/dts/fw_payload_earlycon.hex; then
    echo "OK: CPIO magic found in final hex file"
else
    echo "WARNING: CPIO magic NOT found in final hex - initramfs may not be embedded!"
fi

# ================================================================
# Step 6: Compile Device Tree Blob
# ================================================================
echo ""
echo "=== Step 6: Compiling DTB ==="

if [ -f /output/dts/riscv_soc.dts ]; then
    dtc -I dts -O dtb -o /tmp/riscv_soc.dtb /output/dts/riscv_soc.dts 2>&1 || true
    if [ -f /tmp/riscv_soc.dtb ]; then
        # Convert to LE hex for Verilog $readmemh
        xxd -e -c 4 /tmp/riscv_soc.dtb | awk '{print $2}' > /output/dts/riscv_soc_dtb_le.hex
        echo "DTB compiled: $(wc -l < /output/dts/riscv_soc_dtb_le.hex) hex words"
        echo "Bootargs:"
        strings /tmp/riscv_soc.dtb | grep -i earlycon || echo "  (not found)"
        cp /tmp/riscv_soc.dtb /output/dts/riscv_soc.dtb
    else
        echo "WARNING: DTB compilation failed, using existing hex"
    fi
else
    echo "WARNING: /output/dts/riscv_soc.dts not found"
fi

echo ""
echo "=============================================="
echo "  Build Complete!"
echo ""
echo "  Firmware: /output/dts/fw_payload_earlycon.hex"
echo "  DTB:      /output/dts/riscv_soc_dtb_le.hex"
echo "  Config: SMP=n, BLOCK=n, HVC_SBI=y"
echo "=============================================="
