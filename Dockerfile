FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    wget \
    xz-utils \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Download xPack RISC-V toolchain (supports rv32)
RUN wget -q https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v14.2.0-3/xpack-riscv-none-elf-gcc-14.2.0-3-linux-x64.tar.gz \
    && tar xf xpack-riscv-none-elf-gcc-14.2.0-3-linux-x64.tar.gz -C /opt \
    && rm xpack-riscv-none-elf-gcc-14.2.0-3-linux-x64.tar.gz

ENV PATH="/opt/xpack-riscv-none-elf-gcc-14.2.0-3/bin:${PATH}"

WORKDIR /build
