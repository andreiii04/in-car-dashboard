# In-Car Dashboard

A standalone telemetry display for a car: a custom 4-layer STM32 board with a 2.8" TFT, a 6-axis IMU, an environmental sensor and a GPS module. It powers from the car's 12 V socket through an ordinary USB-C car charger, starts a fresh session on every power-on with no buttons, shows live driving numbers, and writes a summary of each drive to an SD card.

![Board, 3D view from the top](docs/3Dtop.jpeg)

## What it shows and records

**Live, on the screen**
- A G-meter: a dot for the acceleration you feel, its magnitude, and the session's hardest braking, acceleration and cornering each way around the ring
- Speed, session maximum and distance (GPS)
- Temperature, humidity and pressure (BME680), with the board's own heat taken off the temperature
- Air quality as an IAQ index (BME680 through Bosch's BSEC), coloured by the datasheet's air-quality bands, with BSEC's calibration progress next to it until it is done
- Local time, time since power-up, satellites, position and altitude (GPS fused with the barometer)

**Per drive, on the SD card** — one file, `SESSIONS.JSON`, a single valid JSON document with one fixed-size block per drive:
- Start time (UTC and local), duration, moving time, distance, top speed
- Hardest acceleration, braking and cornering, worst lateral load against the car's rollover limit
- Peak pitch and roll and how close they came to the car's limits
- Temperature, pressure and humidity min/max/average; climb and descent
- Average IAQ, CO2 equivalent and breath-VOC equivalent, counting only the samples BSEC called calibrated
- A `faults` section with every init result and error counter, so a drive that went wrong says so in its own record

The record is rewritten in place every 10 s, because the car cuts power without warning. A cut can only ever damage the current drive's block, and the two writes that move the closing bracket are ordered so an interrupted one repairs itself on the next boot.

## Hardware

| Part | Role | Interface |
|---|---|---|
| STMicroelectronics STM32F446RCT6 | Cortex-M4F MCU, 180 MHz, 256 KB flash, 128 KB RAM | — |
| STMicroelectronics LSM6DSOTR | 6-axis accelerometer + gyroscope | I2C1, data-ready on INT1 |
| Bosch Sensortec BME680 | Temperature, pressure, humidity, gas; air quality through Bosch BSEC | I2C1 |
| Quectel L76L-M33 | GPS + GLONASS receiver, 5 Hz NMEA (1 Hz fallback) | USART1, 115200 baud (boots at 9600) |
| GPS ANT-1 chip antenna | Passive GNSS antenna | RF_IN |
| KMRTM28028-SPI / MSP2807 | 2.8" 320×240 TFT, ILI9341 driver, full-size SD socket | SPI1 (display), SPI2 (SD) |
| Microchip MCP1825ST-3302E | 3.3 V LDO | — |
| GCT USB4930-00-A | USB-C: power, and a USB CDC debug console | USB OTG FS |
| Keystone 3003 | Coin-cell holder for the GPS backup supply | V_BCKP |

Protection on the 5 V input: a series Schottky against reverse polarity, a TVS diode and a 220 µF tantalum bulk capacitor. Five expansion headers (I2C, SPI3/I2S3, USART6, CAN1, two 32-bit timer channels) carry the protected 5 V rail for future add-on boards. Flashing and debugging go through a 4-pin SWD header with an ST-Link.

- **PCB:** 4-layer FR4, 1.6 mm, ENIG (two LGA sensors), 61.6 × 95.79 mm. Solid ground on In1, 3.3 V pour on In2. Made at JLCPCB.
- **Schematic:** three sheets, pictured below, or open the KiCad project.
- **Parts list:** [`docs/bom.csv`](docs/bom.csv), with manufacturer part numbers.

<p>
  <img src="docs/3Dside.jpeg" width="49%" alt="Board, side view">
  <img src="docs/3Dbottom.jpeg" width="49%" alt="Board, bottom view, with the IMU on the underside">
</p>

**Display mounting.** The display module is not soldered to the board and does not appear in the renders: female 2.54 mm sockets are soldered into all 18 LCD1 positions (the 14-pin display and touch row and the 4-pin SD row), and male headers on the module's own PCB plug into them, so the screen stacks face-up on top of the main board and can be lifted off. The board's four M3 holes match the module's 44 × 76.08 mm hole pattern, so one set of screws through the lid clamps the display, printed spacer tubes and board together (see Case).

### Case

A two-part 3D-printed enclosure. The bottom tray holds the board on four bosses, with the USB-C opening on one side, intake vents at the BME680 end and exhaust vents at the far end. The lid carries the display window and a notch for the SD card. Four printed spacer tubes sit between the board and the display, and four countersunk self-tapping screws clamp the whole stack from the lid down into the bosses, so the only visible fasteners are flush in the lid. The lid stays clear of the lifted GPS antenna. The tray is printed and test-fitted; the lid and spacers are modelled but not printed yet.

Sources (FreeCAD 1.1): [`bottom_tray.FCStd`](hardware/case/bottom_tray.FCStd), [`top_case.FCStd`](hardware/case/top_case.FCStd), [`spacer_tube.FCStd`](hardware/case/spacer_tube.FCStd), with a STEP export of each next to it for other CAD tools. Dimensions, load path and print settings are in [`docs/dash_case.md`](docs/dash_case.md).

![The case as modelled: lid on the tray, without the board or display](docs/case_assembly.jpeg)

![Board in the bottom tray](docs/case_pcb_side.jpeg)

### Schematic and layout

![Schematic sheet 1: MCU, power input, USB-C and SWD](docs/stm32_main_page.jpeg)

![Schematic sheet 2: IMU, environmental sensor, GPS, backup cell and expansion headers](docs/sensors.jpeg)

![Schematic sheet 3: display and SD card module](docs/display.jpeg)

![PCB layout, top view](docs/pcb.jpeg)

### Known issues on this revision

- **No copper keepout under the GPS antenna** *(fixed in the files)*. The antenna's datasheet asks for a copper-free corner on all layers; on the boards as made it was missed. With solid copper underneath, the module heard nothing, so on the assembled board the chip antenna is lifted about 1 cm above its pads on two wire stubs; it fixes with 9–16 satellites that way. The KiCad files here already carry the keepout, a 13.2 × 20 mm copper-free area on all four layers under the antenna, so the layout picture above shows the corrected board.
- **The display module never drives its SDO line,** so register reads return `0xFF`. The firmware treats the ID check as advisory; writes work normally.
- **The IMU sits on the underside of the board,** so the firmware's axis map flips two axes. This is by design, but it surprises anyone reading raw values.

## Firmware

C on STM32 HAL (CubeMX-generated init) and FreeRTOS through CMSIS-RTOS2, built with CMake and arm-none-eabi-gcc.

A boot task runs the whole init, then starts one task per job and ends. From highest priority to lowest:
- **watchdog** — reloads the hardware watchdog every 250 ms, but only while every other task keeps checking in on time
- **IMU** — woken by the IMU's own data-ready interrupt at about 105 Hz; each sample is mapped from sensor axes to vehicle axes (ISO 8855: X forward, Y left, Z up), run through the attitude filter, then handed to the trip statistics. It is the only task that touches the filter, the statistics and the altitude; GPS fixes and BME680 samples reach it through queues
- **GPS** — drains the UART ring buffer every 10 ms and parses NMEA sentences
- **environment** — runs the BME680 at the times and heater settings Bosch's BSEC asks for (every 3 s), sleeps while it measures, and hands BSEC's learned state to the logger every 10 minutes
- **display** — redraws at 20 Hz, band by band, with no framebuffer
- **console** — the once-a-second status block over USB
- **logger** — rewrites the session record on the SD card every 10 s, and saves BSEC's state

Each task publishes what it knows into one shared snapshot behind a mutex, and the display, console and logger read copies of it. A card write can take up to 250 ms; with the logger at the bottom, it no longer delays a single IMU sample.

| Stack | Modules |
|---|---|
| Sensors | `i2c_bus` → vendor drivers (ST `lsm6dso`, Bosch `BME68x`) → platform glue → `imu`, `env`; `air` (Bosch BSEC: IAQ, eCO2, breath VOC, heat-compensated temperature and humidity) |
| GPS | `uart_bus` (interrupt-driven ring buffer, baud switched at runtime) → `nmea` (parser and PMTK command builder) → `gps` |
| Fusion and statistics | `attitude` (complementary filter, full Euler kinematics, mount rotation), `trip` (distance, peaks, lean, rollover margins, GPS-dropout coasting), `elevation` (GPS altitude fused with barometric pressure) |
| Display | `lcd` (ILI9341 over SPI) → `gfx` (band renderer, dirty rectangles) → `screen` |
| Storage | `sd_spi` (SD SPI mode, CRC on every command and block) → FatFs → `session_record` → `session_log`; `air_slot` → `air_store` (BSEC's learned state in `BSEC.BIN`, two CRC-checked slots written in turn) |
| Console | `usb_serial` (printf over USB CDC) → `console` |
| RTOS | `app_tasks` (boot sequence and task loops), `app_state` (shared snapshot, queues, I2C mutex), `watchdog` (per-task check-ins) |

Everything with no hardware in it is host-testable and tested on a PC, 300 checks across eight modules: `nmea`, `attitude`, `trip`, `elevation`, `gfx`, `sd_crc`, `session_record`, `air_slot`. Expected values come from independent derivations rather than restatements of the code under test. Car- and mount-specific constants (axis map, calibration, wheelbase, track, centre-of-gravity height) live in one header, `firmware/App/fusion/vehicle_axes.h`, explained in [`docs/vehicle_info.md`](docs/vehicle_info.md).

Some decisions worth knowing before changing anything:
- **Pitch and roll come from accelerometer and gyroscope fused.** The car's own acceleration is subtracted first, using GPS speed and speed × yaw rate, so braking and cornering don't read as tilt.
- **GPS speed below 10 km/h is treated as standing still.** A poor fix wanders by several km/h at rest, so the floor is applied once, where the fix enters the trip statistics.
- **Climb and descent come from the barometer alone.** The GPS height after a cold start drifts by up to hundreds of metres over minutes, and counting that as climb was a real bug.
- **The mount is calibrated in the car,** with the board's rest reading measured in both directions on the same spot and averaged, so a sloping parking space cancels out.

The reasoning behind these and most other choices, written down while I learned embedded from scratch, is in [`docs/notes.md`](docs/notes.md); code comments that say "see docs/notes.md" point there.

## Repository layout

```
firmware/            STM32 firmware (CMake, CubeMX project dashboard_firmware.ioc)
  App/               hand-written code, one folder per subsystem:
    sensors/         I2C bus, LSM6DSO and BME680
    gps/             UART ring, NMEA parser, GPS module
    fusion/          pitch/roll, trip stats, elevation, the car's constants
    display/         ILI9341 driver, band renderer, screen layout
    storage/         SD card over SPI, session record and log
    console/         USB serial console
    rtos/            FreeRTOS tasks, shared state, watchdog
    drivers/         vendor sensor drivers (ST, Bosch), unmodified; BSEC goes in drivers/bsec/
    fonts/           generated bitmap fonts
  Core/ Drivers/ FATFS/ Middlewares/ USB_DEVICE/   CubeMX- and ST-generated
  tests/             host tests, each file carries its own compile line
  tools/             font generator
hardware/
  pcb/               KiCad 10 project in_car_dashboard
  libraries/         custom symbols, footprints and 3D models used by the board
  case/              FreeCAD sources and STEP exports of the tray, lid and spacer tubes
docs/                case design note, the car's numbers and what each one is used for, my notes
                     from building it, schematic and board pictures, renders, BOM
```

## Building and flashing

Requirements: CMake ≥ 3.22, Ninja, and `arm-none-eabi-gcc` on the `PATH` (STM32CubeCLT provides all three).

```sh
cd firmware
cmake --preset Debug
cmake --build build/Debug
```

The output is `build/Debug/dashboard_firmware.elf`.

**Air quality needs Bosch's BSEC2 library, which is not in this repository** (it is closed source under Bosch's licence). Without it the firmware still builds and runs: the BME680 gives raw temperature, humidity, pressure and gas resistance, and the IAQ field stays empty. To add it, download **BSEC2 2.6.1.0** from Bosch Sensortec's BSEC page (a licence click-through), unzip it and copy the whole package to `firmware/App/drivers/bsec/`, so that `firmware/App/drivers/bsec/algo/bsec_IAQ/bin/gcc/Cortex_M4F/libalgobsec.a` exists. CMake finds it on the next configure (`BSEC found - air quality on`) and links the Cortex-M4F hard-float build with the `bme680_iaq_33v_3s_4d` config. The folder is in `.gitignore`.

Flash it over SWD with an ST-Link: from an IDE (CLion with the STM32 plugin, or STM32CubeIDE), or with STM32CubeProgrammer. Without a probe, the chip's own USB bootloader works over the same USB-C cable: hold BOOT0 high while plugging in, then

```sh
cd firmware
arm-none-eabi-objcopy -O binary build/Debug/dashboard_firmware.elf firmware.bin
dfu-util -l
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```

`dfu-util -l` should list `@Internal Flash`. The serial console is the USB-C port itself, at `/dev/cu.usbmodem*` on macOS; the baud rate is ignored.

Host tests build with any C11 compiler. For example:

```sh
cd firmware
cc -std=c11 -Wall -Wextra -I App/fusion -I App/gps tests/test_trip.c App/fusion/trip.c -lm -o /tmp/test_trip && /tmp/test_trip
```

Each file in `tests/` has its own line in its header.

## Opening the hardware

- **PCB:** open `hardware/pcb/in_car_dashboard.kicad_pro` in KiCad 10. The project's symbol and footprint paths are relative to the project, so the custom parts in `hardware/libraries/` resolve on any machine; everything else comes from KiCad's standard libraries. The 3D models of the custom parts are not included, since they come from third parties: the board opens, checks and exports normally, and those parts just show without a body in the 3D viewer. To get them back, download the STEP models from the manufacturers and put each in `hardware/libraries/<library>/<library>.3dshapes/` under the file name its footprint asks for.
- **Case:** open the `.FCStd` files in `hardware/case/` in FreeCAD 1.1, or the `.step` files in any CAD tool. The tray prints floor down, the lid top face down and the spacer tubes upright, all without supports.

## Datasheets

Not included in the repository; all are available from the manufacturers:
- STM32F446RC (STMicroelectronics)
- LSM6DSO (STMicroelectronics)
- BME680 (Bosch Sensortec)
- L76L-M33 datasheet and its GNSS protocol specification (Quectel)
- ILI9341 (Ilitek)
- MSP2807 / KMRTM28028-SPI module drawing
- MCP1825 (Microchip)
- USB4930 / USB4940 (GCT)
- 3003 coin-cell holder (Keystone)
- PMEG10020AELP and PMEG3050EP Schottky diodes (Nexperia)
- SMBJ5.0A TVS diode (SMBJ series)
- GPS ANT-1 chip antenna
- SD Physical Layer Simplified Specification v2.00 (SD Association)

## Status

Working on the bench and in the car. Bring-up is complete: console, sensors, calibration, GPS, attitude, interrupt-paced sampling, display, SD logging, the main loop, its move onto FreeRTOS, and air quality through Bosch's BSEC. The first long drive, 118.5 km, matched the route distance and ran with every fault counter at zero.

Still to do:
- Tuning the filters against logged drives
- Printing and fitting the lid
- A second screen for pitch and roll off-road

## License

The hardware design, case, firmware and documentation in this repository are released under the [MIT License](LICENSE).

The custom symbols and footprints in `hardware/libraries/` were drawn from, or adapted from, manufacturer and EasyEDA / SnapEDA sources; their 3D models are not redistributed here.

Third-party code keeps its own licence:
- **CMSIS:** Apache 2.0, see `firmware/Drivers/CMSIS/LICENSE.txt`
- **STM32F4 HAL:** BSD-3-Clause, see `firmware/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt`
- **STM32 USB Device Library:** ST SLA0044, see `firmware/Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt`
- **FatFs R0.12c:** ChaN's FatFs licence, in its source header
- **Bosch BME68x SensorAPI:** BSD-3-Clause
- **ST LSM6DSO driver:** BSD-3-Clause, from ST's `lsm6dso-pid` repository
