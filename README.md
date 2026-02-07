# Pico C/C++ Firmware Template

Embedded systems · Control · Debugging ⚙️  
I/O 🔌, firmware 🔧, and systems that actually run 🚀

---

## Overview

This repository provides a **clean, reproducible template** for embedded firmware projects using the Raspberry Pi Pico SDK.

It is designed for:
- low-level C/C++ development
- deterministic builds
- on-target debugging
- minimal magic

---

## Requirements

- Raspberry Pi Pico SDK (`PICO_SDK_PATH` exported)
- CMake ≥ 3.13
- Ninja
- ARM GCC toolchain
- `pico-tools` installed at `~/pico/tools` (for flashing & OpenOCD)


## Supported Targets

- RP2040 (Pico / Pico W)
- RP2350 (Pico 2 / Pico 2 W)

---

## Features

- Out-of-tree builds
- CMake presets (Debug / Release)
- OpenOCD flash target
- SWD debugging ready
- Modular source / lib structure

---

## Project Structure

```text
.
├── src/            # Application code
├── lib/            # Drivers / utilities
├── build/          # Generated build directories
├── CMakeLists.txt
├── CMakePresets.json
└── README.md
```

---

## Build Flow

Presets define the board and build type.  
You configure once, then live in `cmake --build`.

### First-time configure
```bash
cmake --preset pico2w-debug
```

### Incremental build
```bash
cmake --build --preset build-pico2w-debug
```

### Build + flash
```bash
cmake --build --preset flash-pico2w-debug
```

---

## Debugging

1. Start OpenOCD
2. Attach GDB / DAP
3. Set breakpoints and inspect registers / memory

OpenOCD is expected to run on port `:3333` (SWD).

Designed to work cleanly with:
- gdb-multiarch
- Neovim + nvim-dap

---

## Design Principles

- Explicit control over convenience
- Debug-first mindset
- Hardware-driven design
- Minimal abstractions

---

## Author

**axfer-io**  
Embedded systems · Control · Debugging  
Firmware and systems that actually run.

---

## License

MIT