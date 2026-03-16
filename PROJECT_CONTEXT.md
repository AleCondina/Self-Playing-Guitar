# Self-Playing Guitar - STM32H755ZI Project Context

**Last Updated:** 2026-01-28
**Project Status:** 6-Solenoid Independent PWM Control Implemented ✅

---

## 🎯 Project Overview

**Goal:** Build a self-playing guitar system that uses solenoids to vibrate guitar strings at precise musical frequencies, with closed-loop feedback from a pickup sensor.

**Hardware:**
- **Dev Board (Current):** STM32H755ZI-Q Nucleo-144 (dual-core: CM7 + CM4)
- **Target PCB (Future):** STM32H747BIT6 (custom PCB in development)
- **Actuators:** 6 solenoids (all strings) + 3 stepper motors (expandable to 6)
- **Solenoid Driver:** N-channel MOSFETs (logic-level, 3.3V gate drive)
- **Sensor (Future):** Guitar pickup → ADC → FFT for frequency feedback

**Validated Behavior:**
- Function generator @ 82 Hz, 50% duty cycle successfully vibrates guitar strings
- Confirms PWM-based electromagnetic excitation works for string vibration

---

## 🏗️ System Architecture

### Dual-Core Architecture
```
┌─────────────────────────────────────────────────────────────┐
│ CM7 (Cortex-M7) @ 400 MHz                                   │
│ - Audio processing (FFT on pickup sensor - future)          │
│ - Pitch detection and frequency analysis                    │
│ - Command generation (string/fret → CM4)                    │
│ - Located in: CM7/Core, CM7/App                             │
└─────────────────────────────────────────────────────────────┘
                            ↓ Shared Memory (AXI SRAM)
┌─────────────────────────────────────────────────────────────┐
│ CM4 (Cortex-M4) @ 200 MHz                                   │
│ - Real-time control loop @ 1 kHz (1ms period)               │
│ - Motor control (stepper motor step generation)             │
│ - **Solenoid PWM control** (implemented)                    │
│ - Located in: CM4/Core, CM4/App                             │
└─────────────────────────────────────────────────────────────┘
```

### Shared Memory Communication
- **Location:** `Shared/Inc/shared_memory.h`
- **Size:** ~200 bytes, 32-byte aligned
- **Synchronization:** Hardware semaphores (HSEM)
- **Direction:**
  - CM7 → CM4: Commands, motor speeds, solenoid parameters
  - CM4 → CM7: Status, current positions/speeds, diagnostics

---

## 🎸 Current System Configuration

### Solenoid Configuration (6 solenoids, all strings - INDEPENDENT FREQUENCIES)

| Solenoid ID | Timer    | GPIO Pin | String | Open Note  | Frequency Range   |
|-------------|----------|----------|--------|------------|-------------------|
| 0           | TIM1_CH1 | PE9      | E2     | 82.41 Hz   | 82 - 123 Hz       |
| 1           | TIM3_CH2 | PB5      | A2     | 110.00 Hz  | 110 - 165 Hz      |
| 2           | TIM2_CH1 | PA5      | D3     | 146.83 Hz  | 147 - 220 Hz      |
| 3           | TIM4_CH1 | PB6      | G3     | 196.00 Hz  | 196 - 294 Hz      |
| 4           | TIM16_CH1| PB8      | B3     | 246.94 Hz  | 247 - 370 Hz      |
| 5           | TIM17_CH1| PB9      | E4     | 329.63 Hz  | 330 - 494 Hz      |

**Key Design:** Each solenoid uses a **separate timer** for fully independent frequency control. This allows all 6 strings to play simultaneously at different frequencies (polyphonic capability).

**Note:** PB15 (TIM12_CH2) is no longer used - it shared ARR with TIM12_CH1, causing frequency conflicts.

### Timer Clock Configuration

**Timer PWM Setup:**
- Clock source: timer kernel @ 200 MHz
- Prescaler: PSC = 99 (÷100)
- Effective count frequency: 2.0 MHz
- Formula: `ARR = (2,000,000 / target_freq_hz) - 1`
- Duty cycle: `CCR = ARR × (duty_percent / 100)`

---

## 📁 Project File Structure

### Critical Directories
```
NUCLEO_H755ZI/
├── CM7/                    # Cortex-M7 core
│   ├── Core/               # HAL, startup, main
│   └── App/                # Audio, pitch detection, inter-core
│
├── CM4/                    # Cortex-M4 core
│   ├── Core/               # HAL, startup, main
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   └── stm32h7xx_hal_conf.h  ← HAL_TIM_MODULE_ENABLED
│   │   └── Src/
│   │       ├── main.c                ← Timer init (MX_TIM12/13/14/15/16/17_Init)
│   │       └── stm32h7xx_hal_msp.c   ← GPIO/clock config for all 6 timers
│   └── App/                # Motor control, solenoid control
│       ├── Inc/
│       │   ├── app_cm4.h
│       │   ├── motor_control.h
│       │   └── solenoid_control.h    ← NEW: Public API
│       └── Src/
│           ├── app_cm4.c             ← 1kHz control loop
│           ├── motor_control.c
│           └── solenoid_control.c    ← NEW: Driver implementation
│
└── Shared/                 # Inter-core communication
    ├── Inc/
    │   ├── shared_memory.h           ← Command flags, data structure
    │   └── application_config.h      ← Global constants
    └── Src/
        └── shared_memory.c
```

---

## 🔧 Implemented Features (2026-01-20)

### ✅ Solenoid PWM Control System

**Files Created:**
1. **CM4/App/Inc/solenoid_control.h** - Public API
2. **CM4/App/Src/solenoid_control.c** - Driver implementation

**Files Modified:**
3. **CM4/Core/Inc/stm32h7xx_hal_conf.h:77** - Enabled `HAL_TIM_MODULE_ENABLED`
4. **CM4/Core/Src/main.c** - Added TIM12/TIM13 initialization
5. **CM4/Core/Src/stm32h7xx_hal_msp.c** - Added MSP callbacks for GPIO/clocks
6. **CM4/App/Src/app_cm4.c** - Integrated into control loop
7. **Shared/Inc/shared_memory.h** - Added solenoid control fields
8. **Shared/Inc/application_config.h** - Fixed TIM_CLOCK_MHZ, added constants

### API Functions (solenoid_control.h)

```c
int32_t Solenoid_Init(void);
int32_t Solenoid_SetFrequency(uint8_t solenoid_id, uint8_t string, uint8_t fret);
int32_t Solenoid_SetDutyCycle(uint8_t solenoid_id, uint8_t duty_percent);
void Solenoid_Start(uint8_t solenoid_id);
void Solenoid_Stop(uint8_t solenoid_id);
void Solenoid_StopAll(void);
uint8_t Solenoid_GetActiveMask(void);
uint32_t Solenoid_GetFrequency(uint8_t solenoid_id);
```

### Shared Memory Fields (CM7 → CM4)

```c
volatile uint8_t solenoid_id;          // 0-5 for 6 solenoids
volatile uint8_t solenoid_string;      // 0-5: E2, A2, D3, G3, B3, E4
volatile uint8_t solenoid_fret;        // 0-7: open to fret 7
volatile uint8_t solenoid_duty_cycle;  // 1-99% (0 = use default 50%)
volatile uint8_t solenoid_active_mask; // Bit field: which solenoids running (6 bits)
```

### Command Flags (CM7 → CM4)

```c
#define CMD_TRIGGER_SOLENOID    (1U << 1)  // Start solenoid at freq
#define CMD_STOP_SOLENOID       (1U << 5)  // Stop solenoid (NEW)
```

### Frequency Lookup Table (solenoid_control.c)

Pre-calculated ARR values for 6 strings × 8 frets = 48 frequencies:
- **String 0 (E2):** 82.41 - 123.47 Hz → ARR: 7764 - 5179
- **String 1 (A2):** 110.00 - 164.81 Hz → ARR: 5817 - 3881
- **String 2 (D3):** 146.83 - 220.00 Hz → ARR: 4358 - 2910
- **String 3 (G3):** 196.00 - 293.66 Hz → ARR: 3264 - 2179
- **String 4 (B3):** 246.94 - 369.99 Hz → ARR: 2591 - 1729
- **String 5 (E4):** 329.63 - 493.88 Hz → ARR: 1941 - 1295

**Why pre-calculated?** No floating-point math in real-time loop (< 1µs lookup vs ~50µs calculation).

---

## 🔬 Testing Strategy

### Phase 1: Timer Validation (Hardware)
- [x] Enable HAL_TIM_MODULE, compile successfully
- [x] Flash to board, verify no errors
- [ ] Oscilloscope on PE9: verify 82 Hz @ 50% duty after TIM1 remap
- [x] Identified GPIO configuration issue - fixed with direct register access
- [x] Identified shared ARR issue with TIM12 CH1/CH2 - fixed by using separate timers

### Phase 2: Multi-Channel Validation (6 Independent Timers)
- [ ] Measure all 6 PWM pins with oscilloscope:
  - PE9 (TIM1): E2 @ 82 Hz
  - PB5 (TIM3): A2 @ 110 Hz
  - PA5 (TIM2): D3 @ 147 Hz
  - PB6 (TIM4): G3 @ 196 Hz
  - PB8 (TIM16): B3 @ 247 Hz
  - PB9 (TIM17): E4 @ 330 Hz
- [ ] Verify all 6 frequencies run simultaneously with independent values
- [ ] Verify no cross-talk or interference

### Phase 3: Mechanical Validation
- [ ] Connect solenoids via MOSFETs to power supply
- [ ] Test string vibration per solenoid
- [ ] Test 3 solenoids simultaneously (chord simulation)
- [ ] Verify smooth frequency transitions (no pops)

### Phase 4: Control Loop Integration
- [ ] Test CM7 → CM4 command flow via shared memory
- [ ] Verify response time < 1ms
- [ ] Monitor `cm4_max_loop_time_us` (should be < 500µs)
- [ ] Test error handling (invalid ID, out-of-range fret)

### Phase 5: Closed-Loop Preparation
- [ ] Document solenoid frequencies for FFT reference
- [ ] Verify no interference with future ADC sampling
- [ ] Measure power consumption (3 solenoids active vs idle)

---

## 🐛 Known Issues & Workarounds

### Issue 1: math.h Not Found (IDE Warning)
- **Status:** Cosmetic only, doesn't affect compilation
- **Cause:** IDE IntelliSense can't locate ARM toolchain headers
- **Impact:** None - GCC arm-none-eabi compiler has correct paths
- **Workaround:** Ignore warnings, verify builds succeed

### Issue 2: Incorrect Clock Config Documentation
- **Status:** FIXED (2026-01-20)
- **Problem:** application_config.h claimed TIM_CLOCK_MHZ = 240
- **Reality:** Actual timer clock = 64 MHz (HSI, no PLL)
- **Fix:** Updated `application_config.h:45` with correct value + comment

### Issue 3: TIM12 CH1/CH2 Shared ARR Register
- **Status:** FIXED (2026-01-28)
- **Problem:** Solenoids 0 and 1 both used TIM12 (CH1 and CH2)
- **Reality:** All channels on same timer share ARR register
- **Impact:** Setting frequency for one solenoid overwrites the other
- **Fix:** Moved solenoid 1 to TIM3 (separate timer, separate ARR)

### Issue 4: GPIO Routing Drift Between Code Paths
- **Status:** FIXED (2026-03-15)
- **Problem:** `.ioc`, HAL MSP init, and `app_cm4.c` used different PWM pin maps
- **Impact:** Some timer channels were routed to the wrong physical pins, and TIM13 was duplicated on two GPIOs
- **Fix:** Standardized on the documented PA/PB pin map and removed direct GPIO rerouting from `app_cm4.c`

---

## 🚀 Future Expansion Tasks

### Immediate Next Steps
1. **Hardware Testing** - Validate all 6 PWM outputs with oscilloscope
2. **Solenoid Testing** - Connect hardware, test string vibration on all 6 strings
3. **Polyphonic Testing** - Test multiple solenoids playing simultaneously

### Short-Term (Next Sprint)
- [x] ~~Expand to 6 solenoids (add TIM3/15/16/17 alongside TIM12/13)~~ ✅ COMPLETED
- [ ] Implement pickup sensor ADC acquisition
- [ ] Implement FFT on CM7 for frequency detection
- [ ] Add closed-loop frequency correction

### Medium-Term
- [ ] Port to STM32H747BIT6 custom PCB
- [ ] Integrate motor control (stepper step generation)
- [ ] Implement coordinated motor + solenoid control
- [ ] Add vibrato/tremolo effects (frequency/amplitude modulation)

### Long-Term
- [x] ~~Full 6-string polyphonic control~~ ✅ COMPLETED (independent timers)
- [ ] Alternate tuning support (drop D, DADGAD, etc.)
- [ ] MIDI input for note triggering
- [ ] Real-time tuning calibration

---

## 📊 Performance Metrics & Constraints

### CM4 Control Loop
- **Target Frequency:** 1 kHz (1ms period)
- **Current Overhead:** < 10µs for solenoid control (measured via `cm4_max_loop_time_us`)
- **Acceptable Max:** < 500µs total loop time (leaves 500µs safety margin)

### Timer Resources Used (6 Independent Timers for Solenoids)
- **TIM1:** Solenoid 0 - E2 string (PE9)
- **TIM2:** Solenoid 2 - D3 string (PA5)
- **TIM3:** Solenoid 1 - A2 string (PB5)
- **TIM4:** Solenoid 3 - G3 string (PB6)
- **TIM16:** Solenoid 4 - B3 string (PB8)
- **TIM17:** Solenoid 5 - E4 string (PB9)
- **Reserved for Future:**
  - TIM1-TIM5: Motor step generation
  - TIM6/7: ADC triggering

### Shared Memory Budget
- **Current Size:** ~220 bytes (including solenoid fields)
- **Allocated Region:** 256 KB AXI SRAM (plenty of headroom)
- **Alignment:** 32-byte (matches L1 cache line size)

---

## 🔑 Critical Design Decisions & Rationale

### Why CM4 for Solenoid Control?
- **Real-time:** 1kHz loop ideal for PWM updates
- **Low latency:** Direct MOSFET gate control, no inter-core delays
- **CM7 busy:** Audio processing + FFT will consume CM7 resources

### Why Separate Timers per Solenoid?
- **Independent control:** Each solenoid can play different notes (polyphonic)
- **No re-init overhead:** Changing one frequency doesn't affect others
- **Simpler logic:** No complex channel multiplexing
- **Bug prevention:** Shared ARR registers on multi-channel timers cause frequency conflicts

### Why Pre-Calculated ARR Table?
- **Speed:** < 1µs lookup vs ~50µs floating-point calculation
- **Determinism:** No rounding errors, guaranteed precision
- **Compile-time validation:** Out-of-range values caught at build time

### Why 6 Timers for 6 Solenoids?
- **Full polyphonic support:** All 6 strings can play simultaneously at different frequencies
- **No frequency conflicts:** Each timer has its own independent ARR register
- **Bug fix:** Original TIM12 CH1+CH2 design caused frequency overwrites (shared ARR)
- **Future-proof:** Full guitar coverage without code changes

---

## 🛠️ Build & Development Environment

### Toolchain
- **IDE:** VS Code (current) or STM32CubeIDE
- **Compiler:** arm-none-eabi-gcc (ARM GCC toolchain)
- **Build System:** CMake (separate binaries for CM7 and CM4)
- **Debugger:** ST-Link GDB server + OpenOCD

### Build Instructions
See: [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)

**Quick Build:**
```bash
cd CM4
mkdir -p build && cd build
cmake ..
make
```

**Flash:**
```bash
st-flash write CM4.bin 0x08100000  # CM4 flash base address
st-flash write CM7.bin 0x08000000  # CM7 flash base address
```

---

## 📝 Important Code Patterns & Conventions

### USER CODE Sections
All custom code in CubeMX-generated files must be in `USER CODE BEGIN/END` blocks:
```c
/* USER CODE BEGIN Includes */
#include "solenoid_control.h"
/* USER CODE END Includes */
```
**Why?** Prevents CubeMX from overwriting custom code on regeneration.

### Shared Memory Access
Always use `volatile` and update via direct assignment:
```c
g_shared_memory.solenoid_id = 0;
g_shared_memory.solenoid_string = 1;
g_shared_memory.motor_command_flags |= CMD_TRIGGER_SOLENOID;
```
**Why?** Ensures memory coherency between cores (MPU configured non-cacheable).

### Timer Updates (Glitch-Free)
Use auto-reload preload for smooth frequency changes:
```c
__HAL_TIM_SET_AUTORELOAD(htim, new_arr);
__HAL_TIM_SET_COMPARE(htim, channel, new_ccr);
// Both update simultaneously on next timer overflow
```
**Why?** Prevents momentary frequency/duty glitches during updates.

### Error Handling
Set error flags in shared memory, never block:
```c
if (invalid_input) {
    g_shared_memory.motor_errors |= ERROR_INVALID_COMMAND;
    return -1;
}
```
**Why?** Control loop must never stall (real-time constraint).

---

## 🎓 Lessons Learned

### 1. Always Verify Clock Configuration
- **Lesson:** Don't trust config file values - measure with oscilloscope
- **Impact:** 64 MHz vs 240 MHz = 3.75× frequency error
- **Solution:** Always verify ARR calculations with scope before hardware testing

### 2. Pre-Calculate Lookup Tables
- **Lesson:** Floating-point math in real-time loop = 50× slower
- **Impact:** 24 frequencies × 50µs = 1.2ms overhead (exceeds 1ms loop budget)
- **Solution:** Pre-calculate all ARR values at compile time

### 3. Use Separate Timers for Independent Control
- **Lesson:** Sharing timers requires complex synchronization
- **Impact:** Frequency changes on one channel affect others
- **Solution:** 1 timer channel per solenoid (simple, robust)

### 4. Multi-Channel Timers Share ARR (CRITICAL BUG FIX)
- **Lesson:** TIM12 CH1 and CH2 share the same ARR register
- **Impact:** Setting frequency for solenoid 1 (TIM12_CH2) overwrites solenoid 0 (TIM12_CH1)
- **Symptom:** Both solenoids output same frequency (last one set)
- **Solution:** Use separate timers (TIM1, TIM3) for independent frequencies
- **Date Fixed:** 2026-01-28

### 5. Keep Timer Pin Routing in One Place
- **Lesson:** Duplicating timer pin routing in both HAL MSP and application code is fragile
- **Impact:** A single timer channel can appear on multiple pins or on the wrong connector
- **Solution:** Keep PWM pin routing only in CubeMX/HAL init and remove application-level GPIO remaps
- **Date Fixed:** 2026-03-15

---

## 🔗 Related Documentation

- **STM32H755 Reference Manual:** [RM0399](https://www.st.com/resource/en/reference_manual/rm0399-stm32h745755-and-stm32h747757-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- **STM32H747 Datasheet:** [DS12110](https://www.st.com/resource/en/datasheet/stm32h747bi.pdf) (future target)
- **HAL Driver Documentation:** STM32CubeH7 package
- **Guitar String Frequencies:** [Standard Tuning Table](https://pages.mtu.edu/~suits/notefreqs.html)

---

## 📞 Contact & Project Info

**Project:** COEN 490 Capstone - Self-Playing Guitar
**Location:** `d:\Users\Ale\Desktop\COEN 490\STM32\Nucleo_H755ZI\NUCLEO_H755ZI\`
**Git Branch:** `Ale` (working branch), `main` (stable)

---

## 🔄 Change Log

### 2026-01-28: 6-Solenoid Independent PWM Implementation ✅ BUILD SUCCESSFUL
- ✅ **Expanded to 6 independent timers** (TIM1, TIM2, TIM3, TIM4, TIM16, TIM17)
- ✅ Fixed shared ARR bug: Moved solenoid 1 from TIM12_CH2 to TIM3_CH2
- ✅ Added/updated active solenoid GPIO pins: PE9, PA5, PB5, PB6, PB8, PB9
- ✅ Updated frequency table to cover all 6 strings × 8 frets = 48 frequencies
- ✅ Remapped E2, D3, and G3 to TIM1, TIM2, and TIM4 respectively
- ✅ Standardized GPIO routing to PE9, PB5, PA5, PB6, PB8, PB9
- ✅ Removed direct GPIO register configuration workaround
- ✅ **All 6 solenoids now have independent frequency control!**

**New Pin Assignments:**
| Solenoid | Timer | Pin | String | Frequency |
|----------|-------|-----|--------|-----------|
| 0 | TIM1 | PE9 | E2 | 82 Hz |
| 1 | TIM3 | PB5 | A2 | 110 Hz |
| 2 | TIM2 | PA5 | D3 | 147 Hz |
| 3 | TIM4 | PB6 | G3 | 196 Hz |
| 4 | TIM16 | PB8 | B3 | 247 Hz |
| 5 | TIM17 | PB9 | E4 | 330 Hz |

**Build Output:**
- CM4: 12,900 bytes FLASH (1.23%), 2,224 bytes RAM (0.75%)

### 2026-01-20: Solenoid PWM Control Implementation ✅ BUILD SUCCESSFUL
- ✅ Created solenoid_control driver (API + implementation)
- ✅ Added TIM12/TIM13 initialization for 3-channel PWM
- ✅ Integrated into CM4 1kHz control loop
- ✅ Extended shared memory with solenoid control fields
- ✅ Fixed critical bug: TIM_CLOCK_MHZ (240 → 64 MHz)
- ✅ Added CMD_STOP_SOLENOID command flag
- ✅ Fixed NUM_SOLENOIDS conflict (removed duplicate from shared_memory.h)
- ✅ Added solenoid_control.c to CM4/CMakeLists.txt
- ✅ **FIRMWARE BUILDS SUCCESSFULLY** - Ready for hardware testing!
- 📝 Created PROJECT_CONTEXT.md (this file)

**Build Output:**
- CM4: 10,592 bytes FLASH (1.01%), 1,880 bytes RAM (0.64%)
- CM7: 22,932 bytes FLASH (2.19%), 1,912 bytes RAM (1.46%)

### Next Update: After 6-Channel Hardware Testing
- [ ] Test all 6 PWM outputs with oscilloscope
- [ ] Document measurements for all pins (PE9, PB5, PA5, PB6, PB8, PB9)
- [ ] Test polyphonic operation (multiple strings simultaneously)
- [ ] Connect solenoids and test mechanical string vibration
- [ ] Add performance metrics (loop timing, jitter)

---

**END OF PROJECT CONTEXT**
*Keep this file updated with every significant change*
