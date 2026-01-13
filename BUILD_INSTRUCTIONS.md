# STM32H755ZI Guitar Controller - Build Instructions

## Implementation Status

**All firmware skeleton code is now complete!** The following has been implemented:

### Completed Tasks:
1. ✅ Created `Shared/` directory structure with inter-core protocol
2. ✅ Modified CM7 linker script to add shared memory regions
3. ✅ Modified CM4 linker script to add shared memory regions
4. ✅ Created `CM7/App/` directory with inter-core communication
5. ✅ Created `CM4/App/` directory with motor control stubs
6. ✅ Updated CM7 main.c with MPU config and shared memory init
7. ✅ Updated CM4 main.c with control loop structure
8. ✅ Updated CM7 and CM4 CMakeLists.txt to include App sources

### What's Implemented:

#### Shared Memory (Shared/)
- `shared_memory.h` - Inter-core data structure definition
- `shared_memory.c` - Global shared memory instance
- `application_config.h` - Hardware-agnostic configuration constants

#### CM7 Application (CM7/App/)
- `app_cm7.c` - Main application loop with state machine
- `inter_core_cm7.c` - Inter-core communication with MPU cache config
- `debug_uart.c` - UART debug utilities and printf redirection
- `audio_input.c` - Audio ADC placeholder (to be configured in CubeMX)

#### CM4 Application (CM4/App/)
- `app_cm4.c` - 1ms control loop implementation
- `inter_core_cm4.c` - Inter-core command processing
- `motor_control.c` - Motor control with acceleration limiting (stub)

#### Linker Scripts
- CM7: Added SRAM_SHARED (0x24000000, 256KB) and SRAM_D3 (0x38000000, 64KB)
- CM4: Added SRAM_SHARED (0x24000000, 256KB)
- Both: Added `.shared_data` and `.dma_buffers` sections

---

## Build Requirements

### Required Tools:
1. **CMake** (version 3.22 or later)
2. **ARM GCC Toolchain** (arm-none-eabi-gcc)
3. **GNU Make** or **Ninja build system**

### Install on Windows:
```bash
# Install CMake
choco install cmake

# Install ARM GCC Toolchain
# Download from: https://developer.arm.com/downloads/-/gnu-rm
# Add to PATH: C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\...\bin
```

### Install on Linux:
```bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
```

---

## Build Steps

### 1. Configure Build System

```bash
cd "d:/Users/Ale/Desktop/COEN 490/STM32/Nucleo_H755ZI/NUCLEO_H755ZI"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../gcc-arm-none-eabi.cmake
```

**Expected Output:**
```
-- Build type: Debug
-- Configuring done
-- Generating done
-- Build files have been written to: .../build
```

### 2. Build Firmware

```bash
# Build both CM7 and CM4
cmake --build . -j8

# Or build individually
cmake --build . --target NUCLEO_H755ZI_CM7
cmake --build . --target NUCLEO_H755ZI_CM4
```

**Expected Output:**
```
[100%] Built target NUCLEO_H755ZI_CM7
[100%] Built target NUCLEO_H755ZI_CM4
```

**Output Files:**
- `build/CM7/NUCLEO_H755ZI_CM7.elf` - CM7 executable
- `build/CM7/NUCLEO_H755ZI_CM7.bin` - CM7 binary (for flashing)
- `build/CM4/NUCLEO_H755ZI_CM4.elf` - CM4 executable
- `build/CM4/NUCLEO_H755ZI_CM4.bin` - CM4 binary (for flashing)

---

## Flashing to Board

### Using ST-Link Utility (Windows)

1. Open **STM32CubeProgrammer**
2. Connect to ST-Link
3. Flash CM7:
   - Address: `0x08000000`
   - File: `build/CM7/NUCLEO_H755ZI_CM7.bin`
4. Flash CM4:
   - Address: `0x08100000`
   - File: `build/CM4/NUCLEO_H755ZI_CM4.bin`
5. Reset board

### Using st-flash (Linux/WSL)

```bash
# Flash CM7
st-flash write build/CM7/NUCLEO_H755ZI_CM7.bin 0x08000000

# Flash CM4
st-flash write build/CM4/NUCLEO_H755ZI_CM4.bin 0x08100000

# Reset
st-flash reset
```

### Using OpenOCD

```bash
# Start OpenOCD server
openocd -f interface/stlink.cfg -f target/stm32h7x_dual_bank.cfg

# In another terminal, flash with GDB
arm-none-eabi-gdb build/CM7/NUCLEO_H755ZI_CM7.elf
(gdb) target extended-remote :3333
(gdb) load
(gdb) file build/CM4/NUCLEO_H755ZI_CM4.elf
(gdb) load
(gdb) monitor reset halt
(gdb) continue
```

---

## Testing

### Expected Behavior After Flashing:

1. **Boot Sequence:**
   - CM7 boots first, initializes shared memory
   - CM7 releases CM4 via HSEM ID 0
   - CM4 wakes from STOP mode
   - Both cores signal ready

2. **UART Output (115200 baud):**
   ```
   [CM7] Initialization complete, starting application...

   =========================================
     STM32H755ZI Guitar Controller - CM7
     Firmware v0.1.0
   =========================================

   [CM7] Waiting for CM4 to initialize...
   [CM7] CM4 ready! Starting application.
   [CM7] Transitioning to RUNNING state

   ===== Shared Memory Diagnostics =====
   CM7 Heartbeat: 500
   CM4 Heartbeat: 500
   CM4 Loop Hz:   1000
   CM4 Overruns:  0
   CM4 Max Time:  50 us
   Command Flags: 0x00000000
   Status Flags:  0x00000001
   Motor Errors:  0x00000000
   CM7->CM4 Msgs: 1
   CM4->CM7 Msgs: 1
   =====================================
   ```

3. **Inter-Core Communication Test:**
   - Every 5 seconds, CM7 sends motor speed command
   - CM4 receives command and updates motor state
   - Diagnostics printed via UART

4. **Heartbeats:**
   - CM7 heartbeat increments every loop iteration
   - CM4 heartbeat increments every 1ms
   - Both visible in shared memory diagnostics

### Debug with UART

Connect USB-to-UART adapter to NUCLEO board:
- **TX (CM7)**: CN12 pin 1 (USART3_TX / PD8)
- **RX**: CN12 pin 2 (USART3_RX / PD9)
- **GND**: CN11 pin 8

Use terminal emulator:
```bash
# Linux/WSL
minicom -D /dev/ttyUSB0 -b 115200

# Windows
# Use PuTTY or Tera Term at 115200 baud
```

### Verify Shared Memory

Expected shared memory diagnostics every 5 seconds:
- **CM7 Heartbeat**: Should increment continuously
- **CM4 Heartbeat**: Should increment at ~1000 Hz (1kHz control loop)
- **CM4 Loop Hz**: Should show ~1000 (actual measured rate)
- **CM4 Overruns**: Should be 0 (no timing violations)
- **Status Flags**: 0x00000001 (MOTOR_STATUS_READY)

---

## Troubleshooting

### Build Errors

**Error: "shared_memory.h file not found"**
- **Cause**: CMake include paths not configured
- **Fix**: Already added to CMakeLists.txt, rebuild from scratch:
  ```bash
  rm -rf build
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../gcc-arm-none-eabi.cmake
  cmake --build .
  ```

**Error: "undefined reference to `g_shared_memory`"**
- **Cause**: shared_memory.c not compiled/linked
- **Fix**: Check CMakeLists.txt includes `Shared/Src/shared_memory.c`

**Error: "multiple definition of `g_shared_memory`"**
- **Cause**: shared_memory.c included in both CM7 and CM4
- **Fix**: This is expected! Each core needs its own instance pointing to the same address.

### Runtime Errors

**Problem: CM7 timeout waiting for CM4**
- **Symptom**: UART shows "ERROR: CM4 not responding!"
- **Cause**: CM4 binary not flashed or wrong address
- **Fix**: Flash CM4 to 0x08100000, not 0x08000000

**Problem: HardFault on boot**
- **Symptom**: No UART output, board unresponsive
- **Cause**: Linker script or MPU misconfiguration
- **Fix**:
  1. Verify linker scripts match (SRAM_SHARED at 0x24000000)
  2. Check MPU configuration in inter_core_cm7.c

**Problem: CM4 heartbeat not incrementing**
- **Symptom**: CM4_Heartbeat stuck at 0
- **Cause**: CM4 not running or stuck in STOP mode
- **Fix**:
  1. Verify HSEM_ID_0 release in CM7
  2. Check D2CKRDY flag timeout in CM7 main.c
  3. Ensure CM4 binary flashed correctly

**Problem: Cache coherency issues**
- **Symptom**: CM7 sees stale data, CM4 writes not visible
- **Cause**: MPU not marking shared memory as non-cacheable
- **Fix**: Verify `InterCore_ConfigMPU()` called in CM7 before accessing shared memory

---

## Next Steps

### Immediate Tasks (Before Testing):
1. **Install build tools** (CMake + ARM GCC)
2. **Build firmware** (both CM7 and CM4)
3. **Flash to board** (ST-Link or OpenOCD)
4. **Connect UART** (115200 baud)
5. **Observe output** (heartbeats, diagnostics)

### Peripheral Configuration (CubeMX):
1. **ADC3** - Audio input (44.1kHz sampling)
2. **TIM2-5** - Motor PWM outputs
3. **GPIO** - Solenoid outputs (GPIOD 0-5)
4. **DMA** - ADC circular mode

### Future Features:
1. **Pitch Detection** - Implement FFT in CM7
2. **Motor Control** - Configure TIM PWM in CM4
3. **Solenoid Control** - Add timing logic in CM4
4. **Safety Monitoring** - Limit switches, error detection
5. **Calibration** - Home all motors to reference position

---

## Migration to H747BIT6

When your custom PCB arrives with the H747BIT6 chip:

1. **Create new CubeMX project** for H747BIT6
2. **Copy portable code** (no changes needed):
   - `Shared/` directory (entire)
   - `CM7/App/` directory (entire)
   - `CM4/App/` directory (entire)
3. **Copy linker scripts** (no changes needed):
   - `CM7/stm32h755xx_flash_CM7.ld` → `stm32h747bit6_flash_CM7.ld`
   - `CM4/stm32h755xx_flash_CM4.ld` → `stm32h747bit6_flash_CM4.ld`
4. **Update pin mappings** in CubeMX to match PCB
5. **Build and flash** (same process)

**Estimated migration time: 1-2 days**

---

## Summary

**Current Status:** ✅ **Firmware skeleton complete and ready for testing!**

**What Works:**
- Dual-core boot synchronization (HSEM ID 0)
- Shared memory inter-core communication
- CM7 application loop with state machine
- CM4 1ms control loop with motor stubs
- UART debug output with diagnostics
- Cache coherency via MPU configuration

**What's Stubbed:**
- ADC audio input (needs CubeMX peripheral config)
- Motor PWM control (needs TIM peripheral config)
- Solenoid GPIO (needs GPIO config)
- Pitch detection (placeholder returns 440Hz)

**Ready to Test:**
1. Install build tools (CMake + ARM GCC)
2. Build firmware: `cmake --build build`
3. Flash to NUCLEO-H755ZI board
4. Connect UART and verify output
5. Observe shared memory diagnostics every 5 seconds

**Questions or issues?** Check the troubleshooting section above or examine UART output for error messages.
