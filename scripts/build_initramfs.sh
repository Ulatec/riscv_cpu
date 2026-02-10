#!/bin/bash
# Build minimal initramfs with busybox for RV32IMA
set -e

WORKDIR=/work
CROSS=riscv32-linux-musl-

cd $WORKDIR

echo "=== Building Busybox for RV32IMA ==="

# Download busybox if not present
if [ ! -d "busybox" ]; then
    git clone --depth 1 https://git.busybox.net/busybox
fi

cd busybox

# Clean previous build
make clean || true

# Configure for minimal static build
make ARCH=riscv CROSS_COMPILE=$CROSS defconfig

# Enable static linking and disable problematic features
sed -i 's/# CONFIG_STATIC is not set/CONFIG_STATIC=y/' .config
sed -i 's/CONFIG_HWCLOCK=y/# CONFIG_HWCLOCK is not set/' .config
sed -i 's/CONFIG_FEATURE_HWCLOCK_ADJTIME_FHS=y/# CONFIG_FEATURE_HWCLOCK_ADJTIME_FHS is not set/' .config
sed -i 's/CONFIG_FEATURE_HWCLOCK_LONG_OPTIONS=y/# CONFIG_FEATURE_HWCLOCK_LONG_OPTIONS is not set/' .config

# Rebuild config
make ARCH=riscv CROSS_COMPILE=$CROSS oldconfig

# Build
make -j$(nproc) ARCH=riscv CROSS_COMPILE=$CROSS

echo "=== Creating initramfs structure ==="

cd $WORKDIR
rm -rf initramfs
mkdir -p initramfs/{bin,sbin,etc,proc,sys,dev,tmp,root}

# Install busybox
cp busybox/busybox initramfs/bin/
chmod +x initramfs/bin/busybox

# Create symlinks for common commands
cd initramfs/bin
for cmd in sh ash ls cat echo mkdir mount umount ps kill sleep clear pwd; do
    ln -sf busybox $cmd
done
cd ../sbin
for cmd in init halt reboot; do
    ln -sf ../bin/busybox $cmd
done

cd $WORKDIR/initramfs

# Create init script
cat > init << 'INITEOF'
#!/bin/sh

# Mount essential filesystems
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null

# Ensure console is available (may already exist from cpio or devtmpfs)
[ -e /dev/console ] || mknod -m 622 /dev/console c 5 1
[ -e /dev/null ]    || mknod -m 666 /dev/null c 1 3
[ -e /dev/zero ]    || mknod -m 666 /dev/zero c 1 5
[ -e /dev/ttyS0 ]   || mknod -m 666 /dev/ttyS0 c 4 64

# Redirect stdio to serial console if not already connected
if ! [ -t 0 ]; then
    exec 0</dev/ttyS0 1>/dev/ttyS0 2>&1
fi

# Print banner
echo ""
echo "=================================="
echo "  RV32IMA Linux - Shell Ready!"
echo "=================================="
echo ""

# Start shell on serial console
exec /bin/sh
INITEOF
chmod +x init

# Create /etc/passwd and /etc/group
echo "root:x:0:0:root:/root:/bin/sh" > etc/passwd
echo "root:x:0:" > etc/group

echo "=== Creating initramfs cpio archive ==="

cd $WORKDIR/initramfs

# Create device nodes using fakeroot so they appear in the cpio
# The kernel needs /dev/console BEFORE running /init for stdin/stdout/stderr
fakeroot sh -c '
    mknod -m 622 dev/console c 5 1
    mknod -m 666 dev/null c 1 3
    mknod -m 666 dev/zero c 1 5
    mknod -m 666 dev/ttyS0 c 4 64
    find . | cpio -o -H newc
' > ../initramfs.cpio

echo "=== Rebuilding kernel with initramfs ==="

cd $WORKDIR/linux

# Update config to include initramfs
sed -i '/CONFIG_INITRAMFS_SOURCE/d' .config
echo 'CONFIG_INITRAMFS_SOURCE="/work/initramfs.cpio"' >> .config

# Rebuild kernel
make ARCH=riscv CROSS_COMPILE=$CROSS -j$(nproc)

echo "=== Converting to hex format ==="

cd $WORKDIR
${CROSS}objcopy -O binary linux/arch/riscv/boot/Image linux_with_initramfs.bin

# Convert to hex (little-endian, 32-bit words)
xxd -e -c 4 linux_with_initramfs.bin | awk '{print $2}' > linux_initramfs.hex

# Copy to output
cp linux_initramfs.hex /output/dts/

echo ""
echo "=== Done! ==="
echo "Output: /output/dts/linux_initramfs.hex"
echo ""
