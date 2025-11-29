# PomeloCore Firmware Architecture

## Overview

PomeloCore is a sophisticated gamma spectroscopy system firmware built on an Atmel SAML21G18B ARM Cortex-M0+ microcontroller. The system provides real-time radiation detection, multi-channel analysis (MCA), and dosimetry capabilities for scintillation detectors with SiPM readout.

## System Block Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         SYSTEM INITIALIZATION                   │
│  main.c:2163                                                    │
├─────────────────────────────────────────────────────────────────┤
│  • System clock init (DFLL 48MHz or OSC16M)                     │
│  • GPIO configuration (LED, AFE, HV, triggers, coincidence)     │
│  • Load parameters from NVM                                     │
│  • Initialize peripherals (UART, USB, I2C, ADC, DAC, RTC)       │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                        MAIN LOOP (while 1)                      │
│  main.c:2294                                                    │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ USB Handler  │  │ UART Handler │  │  List Output FIFO    │   │
│  │ (CDC Device) │  │ (921600 baud)│  │  Handler             │   │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────────────┘   │
│         │                 │                  │                  │
│         └─────────────────┴──────────────────┘                  │
│                           │                                     │
│                           ▼                                     │
│              ┌────────────────────────┐                         │
│              │  Command Parser        │                         │
│              │  main.c:1896           │                         │
│              │  (JSON protocol)       │                         │
│              └────────────────────────┘                         │
└─────────────────────────────────────────────────────────────────┘
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
         ▼                 ▼                 ▼
┌─────────────────┐ ┌────────────┐ ┌────────────────┐
│ DATA ACQUISITION│ │ PARAMETER  │ │  CALIBRATION   │
│                 │ │ MANAGEMENT │ │  & DIAGNOSTICS │
├─────────────────┤ ├────────────┤ ├────────────────┤
│ • Start/Stop    │ │ • Set HV   │ │ • ADC calib    │
│ • Histogram     │ │ • Threshold│ │ • Temp comp    │
│ • Dosimetry     │ │ • Save NVM │ │ • Ramp gen     │
│ • Config read   │ │ • Load NVM │ │ • HV load test │
└─────────────────┘ └────────────┘ └────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                   INTERRUPT SERVICE ROUTINES                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ GAMMA TRIGGER ISR (main.c:192)                             │ │
│  │ ────────────────────────────────────────────────────────── │ │
│  │ 1. Particle detection (PIN_PA18 external interrupt)        │ │
│  │ 2. Start ADC conversion (peak detector read)               │ │
│  │ 3. Clear peak detector capacitor                           │ │
│  │ 4. Apply spectral deconvolution (pMove filter)             │ │
│  │ 5. Update histogram bins (gammaSpectrum[])                 │ │
│  │ 6. Coincidence logic (monitor PIN_PA20, ACK on PB11)       │ │
│  │ 7. List mode FIFO update                                   │ │
│  │ 8. Statistics accumulation (count, sum, sum²)              │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ RTC PERIODIC ISR (main.c:315)                              │ │
│  │ ────────────────────────────────────────────────────────── │ │
│  │ • Temperature compensation update (every 1s)               │ │
│  │ • SiPM bias voltage adjustment based on temp               │ │
│  │ • HV load regulation                                       │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ SYNCHRONIZER ISR (main.c:172)                              │ │
│  │ ────────────────────────────────────────────────────────── │ │
│  │ • External sync pulse handler (PIN_PB08)                   │ │
│  │ • Marker insertion in FIFO (value 9999)                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ HV LOAD ISR (main.c:336)                                   │ │
│  │ ────────────────────────────────────────────────────────── │ │
│  │ • HV current monitoring (PIN_PB10)                         │ │
│  │ • Automatic boost mode switching                           │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                      HARDWARE CONTROL LAYER                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ HV GENERATOR │  │  ADC/DAC     │  │  TEMPERATURE SENSOR  │   │
│  │ (main.c:543) │  │ (main.c:677) │  │  (main.c:979)        │   │
│  ├──────────────┤  ├──────────────┤  ├──────────────────────┤   │
│  │• TCC PWM     │  │• ADC0 Ch8    │  │• I2C TMP116/TMP451   │   │
│  │• Dual boost  │  │• DAC0/1      │  │• SiPM temp comp      │   │
│  │• HV crowbar  │  │• Threshold   │  │• Auto voltage adj    │   │
│  │• Load sense  │  │• Peak detect │  │                      │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ COINCIDENCE DETECTION (main.c:401)                       │   │
│  ├──────────────────────────────────────────────────────────┤   │
│  │ • Configurable Analog Comparator + CCL logic             │   │
│  │ • Monitor input (PA20) / ACK output (PB11)               │   │
│  │ • Separate coincidence histogram                         │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                      DATA OUTPUT STREAMS                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  • Spectrum histogram (1024 bins, JSON)                         │
│  • Real-time pulse output (UART/USB)                            │
│  • List mode with timestamps                                    │
│  • Dosimetry calculations (CPM, μSv/h)                          │
│  • System telemetry (temp, uptime, status)                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware Platform

### Microcontroller

- **MCU**: Atmel SAML21G18B
- **Core**: ARM Cortex-M0+
- **Clock**: 48 MHz (DFLL) / 16 MHz (OSC16M)
- **Flash**: 256 KB
- **RAM**: 32 KB

### Key Peripherals Used

- **ADC**: 12-bit, used for peak detector readout
- **DAC**: Dual channel for HV control and threshold
- **TCC**: Timer/Counter for PWM HV generation
- **RTC**: Real-time counter for timing and periodic tasks
- **I2C**: Temperature sensor communication
- **UART**: 921600 baud host communication
- **USB**: CDC device class
- **EXTINT**: External interrupt for trigger detection
- **AC/CCL**: Analog comparator and configurable custom logic for coincidence

## Software Architecture

### Main Components

#### 1. Initialization (main.c:2163)

- System clock configuration
- GPIO pin setup for all detector interfaces
- Peripheral initialization (UART, USB, I2C, ADC, DAC, RTC)
- Parameter loading from NVM
- Initial state configuration

#### 2. Main Loop (main.c:2294)

The main loop operates in a continuous polling mode:

- **USB Data Handler**: Processes USB CDC commands
- **UART Data Handler**: Processes UART serial commands
- **List Mode FIFO**: Outputs event data when enabled
- **Sleep Management**: Power optimization when idle

#### 3. Command Interface (main.c:1896)

The system uses a JSON-based protocol over USB/UART:

**Commands:**

- `h` - Get histogram data
- `s` - Get system status
- `c` - Get configuration
- `x` - Start detector (power on)
- `z` - Stop detector (power off)
- `q` - Query parameters (debug)
- Parameter setting commands (multi-byte protocol)

**Response Format:**

```json
{
  "type": "spectrum|system|config",
  "payload": { ... }
}
```

#### 4. Data Acquisition (main.c:1412)

**DAQ Start Sequence:**

1. Clear spectrum arrays
2. Reset counters
3. Configure RTC compare for timed acquisition
4. Enable interrupt-driven data collection

**DAQ Features:**

- Continuous or timed acquisition modes
- 1024-channel histogram
- Separate coincidence histogram
- Real-time statistics (count, sum, sum of squares)

#### 5. Interrupt Service Routines

##### Gamma Trigger ISR (main.c:192)

The heart of the system - triggered on each particle detection:

1. Read ADC value from peak detector
2. Clear peak detector capacitor
3. Apply spectral deconvolution filter (pMove matrix)
4. Bin event in histogram
5. Handle coincidence logic if enabled
6. Update list mode FIFO
7. Accumulate statistics

**Performance Note:** ISR is optimized for minimal latency to handle high count rates.

##### RTC Periodic ISR (main.c:315)

Runs every second for system maintenance:

- Temperature compensation for SiPM bias
- HV load monitoring and regulation
- Timed acquisition stop

##### Synchronizer ISR (main.c:172)

Handles external synchronization pulses:

- Inserts markers (value 9999) in list mode data
- Allows multi-detector time correlation

##### HV Load ISR (main.c:336)

Monitors high voltage system:

- Measures HV current draw
- Switches between normal/boost modes automatically
- Ensures stable detector operation

### Hardware Control Functions

#### High Voltage System (main.c:543, 592, 651)

- **PWM-based boost converter** with dual mode operation
- **HV enable/disable** with crowbar protection
- **Dynamic boost switching** based on load
- **Temperature-compensated SiPM bias** for gain stability

#### ADC/DAC Management (main.c:677, 708)

- **ADC**: Reads peak detector output (12-bit)
- **DAC0**: Controls high voltage level
- **DAC1**: Sets discriminator threshold
- **Calibration support** for linearity correction

#### Temperature Sensing (main.c:979)

- **Supports TMP116 or TMP451** sensors via I2C
- **Automatic detection** of sensor type
- **Real-time compensation** of SiPM operating voltage
- **Temperature coefficient** stored in NVM parameters

#### Coincidence Detection (main.c:401, 428)

- **Hardware-based** using AC and CCL modules
- **Monitor line** (PA20) for incoming coincidence signals
- **ACK line** (PB11) for daisy-chain operation
- **Separate histogram** for coincidence events
- **Configurable enable/disable**

### Data Structures

#### Core Parameters (nvm_params.h:3)

Stored in NVM, contains system configuration:

```c
struct core_params {
    uint8_t version;
    float vDac[2];              // HV DAC calibration
    float iMeas[3];             // Current measurement cal
    uint16_t threshold;         // Discriminator level
    uint8_t sys_outputs;        // Output enable flags
    uint8_t sys_pulseChar;      // Pulse character for list mode
    bool sys_power;             // Power state
    bool sys_coincidence;       // Coincidence mode enable
};
```

#### Physics Parameters (nvm_params.h:28)

Detector-specific calibration data:

```c
struct physics_params {
    uint8_t version;
    float sipm_vMin;            // SiPM minimum voltage
    float sipm_vMax;            // SiPM maximum voltage
    float sipm_v0deg;           // SiPM voltage at 0°C
    float sipm_vTempComp;       // Temperature coefficient
    float ecal[3];              // Energy calibration polynomial
    float uSvph_constant;       // Dose rate conversion
    char detString[64];         // Detector description
    uint8_t tempType;           // Temperature sensor type
};
```

### List Mode Output

The system supports multiple list mode configurations (bit flags in `sys_outputs`):

**UART Outputs:**

- `LIST_UART_PULSE` (bit 0): Simple pulse character per event
- `LIST_UART_FAST_PULSE` (bit 1): Fast pulse in ISR (minimal latency)
- `LIST_UART_ENERGY` (bit 2): Energy value per event
- `LIST_UART_ENERGY_TS` (bit 3): Energy + timestamp

**USB Outputs:**

- `LIST_USB_PULSE` (bit 4): Pulse character via USB
- `LIST_USB_ENERGY` (bit 5): Energy value via USB
- `LIST_USB_ENERGY_TS` (bit 6): Energy + timestamp via USB

### Spectral Deconvolution

The firmware implements a sophisticated **spectral unfolding algorithm** using a probability matrix `pMove[1024][9]`:

1. Each ADC bin has a probability distribution for spreading counts
2. Corrects for detector response function
3. Improves energy resolution
4. Applied in real-time during acquisition (main.c:217-230)

### Parameter Management

#### Load Parameters (main.c:1499)

- Reads `core_params` and `physics_params` from NVM
- Performs version checking and upgrades old formats
- Validates data integrity

#### Apply Parameters (main.c:2136)

- Configures hardware based on loaded parameters
- Sets HV, threshold, output modes
- Enables/disables coincidence

#### Save Parameters

- Writes updated parameters back to NVM
- Preserves calibration across power cycles

### Bootloader Interface

The firmware supports double-tap reset to bootloader (main.c:2411):

- Magic value `0xf01669ef` written to end of RAM
- Allows firmware updates without hardware programmer
- UF2 bootloader compatible (SAM-BA protocol)

## Communication Protocols

### UART Interface

- **Baud Rate**: 921600 bps
- **Format**: 8N1
- **Flow Control**: None
- **Buffer**: 128 bytes circular buffer with interrupt-driven RX
- **Pin Mapping**: PA12 (TX), PA13 (RX) on SERCOM2

### USB Interface

- **Class**: CDC (Communications Device Class)
- **Speed**: Full Speed (12 Mbps)
- **Enumeration**: Acts as virtual serial port
- **Power Detection**: VBUS sensing on PA27

### I2C Interface

- **Mode**: Master
- **Speed**: 100 kHz (standard mode)
- **Devices**: TMP116 or TMP451 temperature sensors
- **Pin Mapping**: Configured in ASF

## Key Features

1. **1024-Channel Spectroscopy**: High-resolution multi-channel analyzer
2. **Real-Time Processing**: All processing in ISR, no polling delays
3. **Temperature Compensation**: Automatic SiPM bias adjustment for stable gain
4. **Coincidence Mode**: Multi-detector anti-coincidence or coincidence
5. **List Mode**: Event-by-event data with timestamps
6. **Dual Interface**: Simultaneous USB and UART operation
7. **Persistent Calibration**: NVM storage of all parameters
8. **Spectral Deconvolution**: Real-time unfolding for improved resolution
9. **Dosimetry**: CPM and dose rate calculations
10. **Flexible Outputs**: Multiple data streaming options
11. **HV Auto-Regulation**: Adaptive boost converter for stable operation
12. **Bootloader Support**: Easy firmware updates

## Power Management

The firmware includes power optimization features:

- **Clock switching**: DFLL (48 MHz) for USB, OSC16M (16 MHz) for standalone
- **Sleep modes**: Can enter standby when inactive
- **Peripheral power gating**: DAC/HV disabled when detector off
- **USB detection**: Automatic enumeration/de-enumeration

## Memory Usage

- **Spectrum Buffer**: 4 KB (1024 × 32-bit)
- **Coincidence Spectrum**: 4 KB (1024 × 32-bit)
- **List Mode FIFO**: 512 bytes (128 × 16-bit + 128 × 32-bit)
- **ASF Framework**: ~50 KB Flash
- **Application Code**: ~20 KB Flash
- **Stack/Heap**: ~8 KB RAM

## Build Configuration

### PlatformIO Settings (platformio.ini)

- **Platform**: atmelsam
- **Board**: saml21_xpro_b (custom board definition)
- **Framework**: None (bare metal with ASF)
- **Optimization**: -Os (size optimization)
- **Bootloader Offset**: Custom linker script

### Compilation Flags

- Standard: C99 with GNU extensions
- Warnings: Maximum (-Wall and many specific warnings)
- Math: ARM CMSIS DSP support
- Async Drivers: All peripherals use callback mode

## Development Notes

### Code Structure

- **Single file architecture**: All application code in main.c (2415 lines)
- **ASF integration**: Atmel Software Framework for peripheral drivers
- **Minimal dependencies**: Optimized for embedded constraints

### Future Enhancements

Potential areas for improvement:

- Modular code organization (separate files for subsystems)
- DMA for ADC/UART to reduce ISR overhead
- SD card support for long-term data logging
- Network interface (WiFi/Ethernet module)
- OLED display support
- Battery management for portable operation

## References

- **Hardware**: PomeloCore PCB schematics in parent directory
- **MCU Datasheet**: SAML21 Family Datasheet (Microchip)
- **ASF Documentation**: Atmel Software Framework API reference
- **Bootloader**: UF2 bootloader for SAMD/SAML (Adafruit/Microsoft)

---

**Document Version**: 1.0
**Firmware Version**: Based on commit 0c74e4e
**Date**: 2025-11-25
**Author**: Generated from source code analysis
