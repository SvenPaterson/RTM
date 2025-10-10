# RTM

Re-write of the Rotary Stand code base.

## Project Overview

The RTM (Rotary Test Machine) controls a rotary seal test stand capable of executing user-defined motion and thermal profiles. Test profiles are stored on an SD card, which the controller reads to automatically run sequences with configurable speeds, accelerations, dwell times, temperature targets, and optional data logging back to the SD card. The current prototype uploads firmware and provides diagnostic serial output over USB, while future revisions aim to add more robust field-debug and validation workflows.

## Hardware Architecture

| Subsystem | Description | Key Details |
|-----------|-------------|-------------|
| Motion controller | **Teknic ClearCore** | Primary control platform handling motor motion, I/O coordination, and power distribution. User manual: [ClearCore User Manual](https://teknic.com/files/downloads/clearcore_user_manual.pdf). |
| Protocol / display controller | **Arduino Nano Every** | Manages UI, protocol parsing, and auxiliary I/O at 5 V logic levels. Communicates with ClearCore over TTL serial (COM1 ↔ RX/TX). |
| Motor | **Teknic ClearPath CPM-SDHP-N0563A-ELN** | Main drive motor for the rotary stand. Documentation: [Model Info](https://teknic.com/model-info/CPM-SDHP-N0563A-ELN_Fan/?model_voltage=230VAC3ph), [Manual](https://teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf). |
| Motor power | **230 VAC, 3φ** | ClearPath motor requires mains-class supply routed through ClearCore-managed safety interlocks. |
| Display | **Newhaven NHD-0420D3Z-NSW-BBW-V3** | 4×20 character LCD display for local status and prompts. Datasheet: [PDF](https://newhavendisplay.com/content/specs/NHD-0420D3Z-NSW-BBW-V3.pdf). |
| Temperature sensing | **MAX31855 thermocouple interface** | Prototype supports one channel; final build supports two thermocouple inputs for temperature feedback. |
| Storage | **microSD card** | Profile storage and optional data logging. Interface level-shifted to 3.3 V using SN74LVC245AN bus transceiver. |
| Level shifting | **SN74LVC245AN** | Provides 5 V ↔ 3.3 V translation between controllers and SD interface. |
| Board-to-board link | **TTL serial (present) → RS-485 (planned)** | Prototype uses ClearCore COM1 at TTL levels; production will adopt RS-485 for ~10 ft harness between controller boards. |

## Power Distribution

* **Primary input:** ClearCore operates from a 24 VDC supply sized for the ClearPath drive and auxiliary loads. Document connector type and current budget as the wiring package is finalized.
* **5 V rail:** ClearCore’s communication port exports a regulated 5 V that powers the Nano Every, LCD, and other low-power peripherals. Budget the aggregate draw of the logic chain to ensure margin on the ClearCore supply.
* **3.3 V domain:** MAX31855 thermocouple boards and the microSD socket require 3.3 V. Use SN74LVC245AN (or equivalent) for 5 V ↔ 3.3 V translation to protect the media.
* **Protection:** Capture plans for inline fusing, transient suppression, and brown-out handling once the enclosure and cable lengths are locked in.

## I/O and Interfaces

* **Serial (controller-to-controller):** ClearCore COM1 ↔ Nano Every UART at TTL levels today; design in RS-485 transceivers for noise immunity across the 10 ft harness.
* **USB:** Firmware upload path for both controllers and a convenient channel for development-time serial diagnostics.
* **Thermocouple inputs:** One MAX31855 populated on the prototype, with footprints for two channels on the production PCB. Capture cold-junction compensation offsets during calibration.
* **LCD interface:** Parallel/SPI (per display configuration) from Nano Every to the Newhaven module. Include contrast potentiometer and backlight control guidance in the wiring diagram.
* **SD card interface:** SPI bus running at 3.3 V logic via the SN74LVC245AN transceiver. Document chip-select usage and any required pull-ups.

## Mechanical Variants

Two mechanical configurations are planned:

1. **Direct-drive variant** – baseline stand configuration without additional gearing.
2. **High-speed variant** – incorporates a 1:3 gearbox to achieve higher rotational speeds for specific test requirements.

Document any resulting differences in motor tuning, acceleration limits, and safety checks as development progresses.

## Assembly and Setup Notes

* Prototype boards are wired for USB connectivity; ensure reliable strain relief for repeated firmware updates and log capture.
* Verify SN74LVC245AN orientation, OE direction, and reference voltages prior to first power to avoid SD card damage.
* Confirm thermocouple polarity and cold-junction compensation behavior when populating the second MAX31855 channel.
* Label connectors for the eventual RS-485 upgrade so that the prototype harness can be re-used with minimal rework.
* Build out a structured bring-up checklist (power rails → inter-board comms → motion → thermal loop → logging) and capture results in the project wiki.

## Programming and Debugging

* Firmware is currently uploaded via USB using the respective vendor toolchains (ClearCore SDK and Arduino IDE/PlatformIO for Nano Every).
* Maintain simultaneous serial-terminal access for both controllers to observe protocol exchanges and watchdog/error output.
* Evaluate adding in-system programming headers (SWD/JTAG) or external debug connectors during the PCB refinement phase to shorten iteration cycles.
* Capture the exact PlatformIO environment, ClearCore firmware revisions, and bootloader versions used for release builds.

## Testing and Validation

* **Motion checkout:** Use the ClearCore USB serial terminal to jog axes, confirm interlock status, and verify that the ClearPath motor spins freely in both mechanical variants (direct-drive and 1:3 gearbox).
* **Thermal calibration:** Exercise the MAX31855 inputs with known temperature references to validate wiring, scaling, and cold-junction offsets before running thermal profiles.
* **Profile + logging:** Validate SD card read/write operations by running a short profile and confirming log-file creation when data capture is enabled.
* **Communications:** Stress-test the Nano Every ↔ ClearCore protocol with induced noise or long cable lengths ahead of the RS-485 transition.

## Safety and Regulatory Notes

* Document emergency-stop wiring, enclosure interlocks, and any guarding once the mechanical build is finalized.
* Track compliance targets (e.g., CE/FCC/UL) and the supporting documentation that must accompany production units.
* Highlight high-voltage areas (24 V distribution and mains-powered ClearPath drives) in wiring diagrams and installation instructions.
* Include safe handling guidance for mains wiring into the ClearPath motor and note required lockout/tagout procedures during maintenance.

## Future Documentation Needs

Additional details to capture as the design matures:

* Power supply specifications (current draw, connector types, protection circuitry).
* Detailed pinouts for interconnect cables and terminal blocks, including planned RS-485 mapping.
* Environmental limits, enclosure design, and mounting information.
* Extended validation procedures, including thermal soak tests and long-duration motion profiles.
* Safety warnings, regulatory considerations, and compliance artifacts.
* Photos, schematics, and wiring diagrams for the final build.
* Long-term data retention strategy for SD card logs and synchronization to higher-level quality systems.

This README will be expanded as more hardware information becomes available and the control software evolves.
