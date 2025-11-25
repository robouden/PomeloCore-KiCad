# PomeloCore - PlatformIO Build Instructions

## Overview

This project has been configured to build with PlatformIO while maintaining the original Atmel Software Framework (ASF) codebase. The source code remains unchanged from the Atmel Studio version.

## Prerequisites

1. **PlatformIO Core** or **PlatformIO IDE**
   - Install via: `pip install platformio` or use VS Code extension
   - Documentation: https://platformio.org/install

2. **ARM GCC Toolchain** (automatically installed by PlatformIO)

## Project Structure

```
PomeloCore/
├── platformio.ini          # PlatformIO configuration
├── extra_script.py         # Post-build UF2 conversion script
├── uf2conv.py             # UF2 converter (from Atmel Studio)
├── uf2families.json       # UF2 family definitions
├── PomeloCore/
│   └── src/               # All source code (unchanged)
│       ├── main.c
│       ├── nvm_params.c
│       ├── config/
│       └── ASF/           # Atmel Software Framework
└── .pio/                  # PlatformIO build output (generated)
```

## Building the Firmware

### Command Line

```bash
# Navigate to project directory
cd "/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/firmware/PomeloCore"

# Build firmware
pio run

# Clean build
pio run -t clean

# Verbose build output
pio run -v
```

### Build Output

After successful build, you'll find:
- **ELF file**: `.pio/build/saml21g18b/firmware.elf`
- **BIN file**: `.pio/build/saml21g18b/firmware.bin`
- **UF2 file**: `PomeloCore.uf2` (in project root)

## Uploading Firmware

### Via UF2 Bootloader (Recommended)

1. Build the firmware: `pio run`
2. Put device in bootloader mode (double-tap reset button)
3. Copy `PomeloCore.uf2` to the UF2 drive that appears
4. Device will automatically reset and run new firmware

### Via SWD/JTAG

```bash
# Using EDBG debugger
pio run -t upload

# Specify upload port
pio run -t upload --upload-port /dev/ttyACM0
```

## Serial Monitor

```bash
# Open serial monitor at 921600 baud
pio device monitor -b 921600

# Or specify port
pio device monitor -b 921600 -p /dev/ttyACM0
```

## Debugging

```bash
# Start debug session (requires EDBG debugger)
pio debug
```

## Configuration Details

### MCU Settings
- **Chip**: ATSAML21G18B
- **CPU Frequency**: 16 MHz (configurable via clocks)
- **Flash**: 256 KB
- **RAM**: 32 KB + 8 KB Low Power RAM

### Build Flags
All compiler flags from Atmel Studio are preserved:
- Optimization: `-Os` (size)
- Callback modes enabled for all peripherals
- USB Device LPM support
- Float printf support

### Linker Script
Uses custom bootloader linker script:
`PomeloCore/src/ASF/sam0/utils/linker_scripts/saml21/gcc/saml21j18b_flash_bootloader.ld`

## Troubleshooting

### Build Errors

**Include file not found**:
- Verify all ASF files are present in `PomeloCore/src/ASF/`
- Check include paths in `platformio.ini`

**Linker errors**:
- Ensure CMSIS math library exists: `PomeloCore/src/ASF/thirdparty/CMSIS/Lib/GCC/libarm_cortexM0l_math.a`
- Verify linker script path is correct

**UF2 conversion fails**:
- Check that `uf2conv.py` exists in project root
- Ensure Python 3 is installed
- Verify `uf2families.json` is present

### Upload Issues

**Device not found**:
- Check USB connection
- Verify device is in bootloader mode
- Try different USB cable/port

**Upload fails**:
- Ensure no other program is using the serial port
- Check upload protocol in `platformio.ini`

## Comparison with Atmel Studio

### What's the Same
- All source code (no changes required)
- Compiler flags and optimization settings
- Linker configuration
- Output binary (functionally identical)
- ASF framework version

### What's Different
- Build system (PlatformIO instead of Atmel Studio)
- Build directory (`.pio/` instead of `Debug/` or `Release/`)
- Configuration file (`platformio.ini` instead of `.cproj`)

## Advanced Usage

### Custom Build Flags

Edit `platformio.ini` to add custom flags:
```ini
build_flags = 
    -DMY_CUSTOM_FLAG=1
```

### Multiple Build Configurations

Create additional environments in `platformio.ini`:
```ini
[env:debug]
build_type = debug
build_flags = -DDEBUG -g

[env:release]
build_type = release
build_flags = -DNDEBUG -Os
```

Build specific environment:
```bash
pio run -e debug
pio run -e release
```

## Migration Notes

This project uses the deprecated Atmel Software Framework (ASF). For long-term maintainability, consider migrating to:
- **Microchip Harmony 3**: Modern, actively maintained framework
- **Arduino SAMD**: Simpler API, good community support

However, the current ASF-based code will continue to work with PlatformIO.

## Support

For PlatformIO issues:
- Documentation: https://docs.platformio.org/
- Community: https://community.platformio.org/

For PomeloCore hardware/firmware:
- Project repository: (add your repo URL here)
