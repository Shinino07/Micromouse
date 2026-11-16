# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Micromouse for UET Makerthon 2026 (16×16 maze, 180 mm cells, 4-cell goal at (7,7)–(8,8), start cell bottom-left facing north).
Hardware: STM32F405RGT6, 5 IR eyes, SPI gyro (ICM-20602 clone, MPU-6500 register map), 2 encoders, 2 DC motors.
The user writes in Vietnamese. READMEs are in Vietnamese with diacritics. Log strings in code are unaccented Vietnamese ASCII.

Each top-level folder is a separate STM32CubeMX **CMake** project (CubeIDE 2.x: File › Import › STM32 CMake Project):
- `logic_code/`: the race firmware (portable C++ core, STM32 port, PC simulator, log reader).
- `test_dc/`, `test_sensor/`, `test_encoder/`: single-peripheral bring-up and measurement firmware. Their measured results feed `logic_code/core/config.h` and `logic_code/stm32/sensor_calib.h`.
- `Structure/micromouse_v2.ioc`: the reference pin configuration for every project.
- `informations/` (git-ignored): competition rules, timeline, qualifier map PDFs.

## Commands

Toolchain: `g++`, `cmake` and `ninja` come from mingw64 on PATH. `arm-none-eabi-gcc` and `STM32_Programmer_CLI` are **not** on PATH. They live inside CubeIDE:
```
GCC=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin
CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
```

Simulator (from `logic_code/`). There is no unit-test framework: the simulator *is* the test suite.
```
g++ -std=c++17 -O2 -o build/sim.exe core/*.cpp sim/sim_main.cpp
build/sim.exe sim/mazes/vongloai.txt [seed] [--quiet]   # AUTO mode, 3 runs on the qualifier maze; last line = KET QUA summary
build/sim.exe --random 20 7                              # 20 random mazes (seed 7) + planner checks
build/sim.exe --random 1 <seed>                          # reproduce a single random maze
build/sim.exe --clear sim/mazes/vongloai.txt             # clearance of every fast-run motion
SIM_MODE=1 build/sim.exe sim/mazes/chu_n.txt             # FOLLOW mode (field-test mode)
```
Realism knobs are environment variables read in `sim/sim_main.cpp` (grep `getenv`). Examples: `SIM_FCAP=138 SIM_FMOVEBIAS=35` reproduce the real front eye, `SIM_NOGYRO`, `SIM_FSPIKE`, `SIM_SIDEFWD`, `SIM_ODO`, `SIM_DY`, `SIM_DUMP=dir`.
Compile-time `MM_*` macros in `config.h` can be overridden with `-DMM_...=...` for A/B experiments.
Maze `.txt` format: 33 lines of `o---o` grid, top row first. A horizontal wall is `-` at column 4x+2 and a vertical wall is `|` at column 4x.

Firmware (from `logic_code/micromouse_fw/`; same recipe for each `test_*`):
```
export PATH="$GCC:$PATH"
cmake --preset Debug          # first time only
cmake --build build/Debug     # -> build/Debug/micromouse_fw.elf
```

Flash (ask the user to lift the robot first: the right motor spins while the chip is held in reset):
```
"$CLI" -c port=SWD mode=UR freq=100 -d build/Debug/micromouse_fw.elf -v -rst      # add -e all to also wipe log + saved map
```
- **Never flash with `mode=HOTPLUG`.** The running ADC DMA overwrites the loader's RAM buffer, which gives verify failures or a core LOCKUP. HOTPLUG is for reads only, for example a live RAM peek with `-r32 <addr> <bytes>` and addresses from `nm` on the elf.
- `freq=100` is needed because the SWD extension wires are long. Plugging or unplugging the ST-Link resets the robot. Real runs are untethered and started with the buttons.
- `python logic_code/tools/readlog.py [all|map]` reads the flash log (sector 10, 0x080C0000) and the saved maze (sector 11, 0x080E0000) over SWD.

## logic_code architecture

`core/` is platform-free C++17 (namespace `mm`). It is compiled unchanged into both the simulator and the firmware. `core/hw.h` declares the few services the core needs (millis, buttons, maze load/save in flash, log, reset). `stm32/port_stm32.cpp` implements them on HAL, and `sim/sim_main.cpp` implements them against a physics model with IR ray casting.

Two execution contexts:
- **1 kHz ISR**: SysTick → `App_Tick1kHz` (port) reads encoders, IR and gyro into `TickIn`, then calls `Mouse::tick` → `Motion::tick`, and writes motor duties from `TickOut`. All control runs here. Keep it bounded.
- **Main loop**: `App_Loop` → `Mouse::loop` runs the state machine (MENU/ARMED/RUN/REST), buttons, explore decisions and route compilation, plus `imu_watchdog()`.
- They communicate only through `Motion`: a ring queue of `Move`s (`push`, `free_slots`), and one-shot results the ISR posts and the loop takes (`take_obs` for wall observations, `take_guard` for the late-front-wall guard, `fault()`).

Pipeline: `maze` (wall bits) → `planner` (`flood` BFS for exploring; `find_route` time-weighted Dijkstra for fast runs) → `Mouse::build_moves` compiles a `Route` into `Move`s using the turn geometry tables in `moves.cpp` (search 90, fast 90/180, 45/135 in/out, V90, diagonals, plus a clearance check against the real body size that drops diagonals and then large turns) → `Motion` runs trapezoidal profiles.

Control in `motion.cpp` is model-based feedforward (`KS·sgn v + KV·v + KA·a`, rotation scaled by `ROT_INERTIA`) plus PID. The PID gains are *derived* in `config.h` from just `WN_POS`/`WN_ANG`/`ZETA`, so tune those, not individual gains. The same file also does side-wall following (with a saturation guard and a distance-domain filter), wall-edge position correction, front alignment from the two diagonal eyes, and heading correction from walls.

`core/config.h` holds every physical constant and tuning flag (geometry, encoder/motor/gyro signs and scales, IR poses and thresholds, run-speed profiles, feature toggles such as `EXPLORE_LOOK`, `EDGE_CORR`, `FOLLOW_CURVE`). The race build uses `LOCK_FOLLOW = false` in `config.h` and `mode_ = Mode::AUTO` in `core/mouse.h`. The field-test build locks to FOLLOW (mode 1).

Modes (`Mode` enum in `mouse.h`): AUTO chains explore → RUN_SAFE → RUN_FAST, resting ≥ 2.6 s at start between runs, and saves the map after each run. The others are FOLLOW, single explore/fast runs, and straight/pivot/IR/erase tests. Buttons: SW3 (PA12) short press = next mode (N beeps = mode N−1), hold ~1 s = start. SW2 (PA11) = start, hold ≥ 2 s = reset. Any button during a run = stop.

The STM32 glue lives in `micromouse_fw/` (CubeMX-generated). It only calls `App_Init`/`App_Loop` in `main.c` and `App_Tick1kHz` in `stm32f4xx_it.c`, inside `USER CODE` blocks. Its `CMakeLists.txt` pulls in `../core/*.cpp` and `../stm32/port_stm32.cpp`. CubeMX regeneration only preserves code inside `USER CODE BEGIN/END` markers.

IR roles (`cfg::IrId`): `IR_F` = S3 front, `IR_DL`/`IR_DR` = S4/S2 diagonals, `IR_L`/`IR_R` = S5/S1 sides. The reading is |emitter on − off| converted by d = k·v^−n. The coefficients in `stm32/sensor_calib.h` are produced by `test_sensor/tools/fit_kn.py`. The emitters are driven in 3 groups (PC2 front, PC1 side, PA5 diag).

## Hardware facts that shape the software

- The front eye S3 is only trustworthy to ~140 mm. Beyond that it reads ~135–145 mm with nothing ahead, and while moving it reads a real wall ~35 mm short. This is why explore uses stop-at-centre + stationary "look" (`EXPLORE_LOOK`, `follow_look`) and creeps by encoder before aligning and pivoting. The user has validated this sensor: adapt the software, don't ask for hardware rework.
- The side eyes saturate below ~41 mm (S5) and ~31 mm (S1). The code then switches to the other side.
- Motors must use slow-decay PWM (one pin held on, the other chopped at 1 − duty).
- The gyro is mounted upside down (`GYRO_SIGN = -1`) and can self-reset on a weak supply. The port re-initialises it via the watchdog and falls back to encoder heading after 3 resets in 3 s. The fallback drifts and is not race-worthy.

## Workflow

- After changing `config.h` or anything in `core/`, rebuild the sim and run both the qualifier maze and `--random 20 7` before building and flashing. Report collisions or failed runs from the `KET QUA` lines.
- `test_dc` is driven over SWD by `test_dc/tools/tune.py` (it prints `__doc__` as usage, so keep that docstring). It stores armed params and results in flash and prints lines to paste into `logic_code/core/config.h`.
- Everything under `logic_code/` (except `micromouse_fw/Drivers/`) is comment-free on purpose: the user had every comment stripped on 2026-11-16. In `micromouse_fw/` only the CubeMX `/* USER CODE BEGIN/END */` markers were kept, and they must stay. Don't add comments there unless asked. The reasoning behind constants lives in `logic_code/README.md`.
- When writing C string literals that contain `\n`, use the Edit/Write tools rather than shell heredocs. Heredocs through the Bash tool turn `\n` escapes into real newlines.
