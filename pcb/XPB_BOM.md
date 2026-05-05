# XPB Daughter Board — Bill of Materials

**Board name:** RTM Expansion Board (XPB)  
**Revision:** rev0 (in-progress design)  
**Date:** 2026-05-05  
**Purpose:** Operator UI, thermocouple sensing, SD card storage, and TTL link to ClearCore.  
Heater PID runs on-board; computed output (0–150 scale) is relayed to ClearCore over TTL — no SSR or relay is on this board.

---

## Microcontroller

| # | Description | Part / Reference | Qty | Notes |
|---|-------------|-----------------|-----|-------|
| U1 | MCU module | Arduino Nano Every (ATmega4809, 16 MHz, 5 V) | 1 | Or bare ATmega4809 with supporting passives if going fully custom. USB-C variant preferred for field use. |

---

## Display

| # | Description | Part / Reference | Qty | Notes |
|---|-------------|-----------------|-----|-------|
| LCD1 | 4×20 character SPI LCD | Newhaven NHD-0420D3Z-NSW-BBW-V3 | 1 | 5 V, SPI Mode 3, max 100 kHz clock. Driven by custom `LCDDriver` via CS on D8. Requires MOSI + SCK (no MISO). |

---

## Sensors

| # | Description | Part / Reference | Qty | Notes |
|---|-------------|-----------------|-----|-------|
| U2 | Thermocouple amplifier — Seal | Maxim MAX31855KASA (SOIC-8) | 1 | SPI read-only. CS on D9. Reads seal-face thermocouple (`latestSealC_`). K-type input. |
| U3 | Thermocouple amplifier — Sump | Maxim MAX31855KASA (SOIC-8) | 1 | SPI read-only. CS on D10. Reads sump/heater thermocouple (`latestSumpC_`). K-type input. PID process variable. |
| J_TC1 | K-type thermocouple connector — Seal | Omega PCC-SMP-K or equiv. panel-mount | 1 | Miniature thermocouple connector, K-type polarity. |
| J_TC2 | K-type thermocouple connector — Sump | Omega PCC-SMP-K or equiv. panel-mount | 1 | Miniature thermocouple connector, K-type polarity. |

> **Note:** Adafruit MAX31855K breakout (#269) can be used for prototyping in place of bare ICs.

---

## Storage

| # | Description | Part / Reference | Qty | Notes |
|---|-------------|-----------------|-----|-------|
| J_SD | MicroSD card socket | Amphenol 101-00660-68 or Molex 5031820800 | 1 | SPI interface. CS on D4. Holds `protocol.csv` and resume slots `RA.BIN` / `RB.BIN`. |
| — | MicroSD card | Any Class 4+ microSD, 2–32 GB | 1 | FAT16/32 formatted. Not a PCB component; operator-supplied. |

---

## User Input

| # | Description | Part / Reference | Qty | Notes |
|---|-------------|-----------------|-----|-------|
| SW1 | RUN pushbutton | SPST momentary NO, panel-mount (e.g., E-Switch LS1231) | 1 | Active-low; uses internal pull-up on D2. 25 ms debounce in firmware (Bounce2). |
| SW2 | RESET pushbutton | SPST momentary NO, panel-mount (e.g., E-Switch LS1231) | 1 | Active-low; uses internal pull-up on D3. 25 ms debounce in firmware (Bounce2). |

---

## Connectors & Wiring

| # | Description | Part / Reference | Qty | Notes |
|---|-------------|-----------------|-----|-------|
| J_TTL | ClearCore TTL link | 3-pin JST-PH 2 mm or equiv. (GND / TX / RX) | 1 | Serial1 on Nano Every: D0 = RX ← ClearCore TX, D1 = TX → ClearCore RX. 9600 baud, 3.3 V logic from ClearCore side. Recommend level-shift if board is 5 V. |
| J_PWR | Power input | 2-pin screw terminal or barrel jack | 1 | 5 V regulated supply (USB) or 7–21 V via Nano Every VIN pin if regulated on-board. |
| J_LCD | LCD interface header | 2×5 or 2×8 0.1" pin header | 1 | MOSI (D11), SCK (D13), CS (D8), VCC, GND (LCD has no MISO). |

---

## Passives

| # | Description | Value / Package | Qty | Notes |
|---|-------------|----------------|-----|-------|
| C1–C4 | Bypass capacitor | 100 nF, 0402 or 0603, 10 V+ | 4 | One per IC: U2 (MAX31855 TC1), U3 (MAX31855 TC2), SD socket VCC, LCD VCC. Place close to VCC pin. |
| C5 | Bulk decoupling capacitor | 10 µF, electrolytic or tantalum, 10 V | 1 | At main 5 V power rail entry. |
| R1, R2 | Switch pull-down (optional) | 10 kΩ, 0402 | 2 | Only needed if button wiring is long/noisy. Internal pull-ups used in firmware; omit if traces are short. |

---

## Pin Assignment Summary (Nano Every)

| Pin | Signal | Peripheral |
|-----|--------|-----------|
| D0 (RX) | TTL RX ← ClearCore TX | Serial1 |
| D1 (TX) | TTL TX → ClearCore RX | Serial1 |
| D2 | RUN switch (active-low, INPUT_PULLUP) | SW1 |
| D3 | RESET switch (active-low, INPUT_PULLUP) | SW2 |
| D4 | SD_CS | SD card |
| D8 | LCD_CS | NHD LCD |
| D9 | TC1_CS | MAX31855 — Seal |
| D10 | TC2_CS | MAX31855 — Sump |
| D11 (MOSI) | SPI MOSI | LCD |
| D12 (MISO) | SPI MISO | SD, TC1, TC2 |
| D13 (SCK) | SPI SCK | LCD, SD, TC1, TC2 |
| VIN / 5V | Power | All peripherals |
| GND | Common ground | All |

> All four SPI CS lines are driven HIGH during `spiQuiesceAll_()` before SD init to prevent MISO bus contention.

---

## Design Notes for PCB / RFQ

1. **SPI bus**: LCD, both MAX31855s, and the SD card share the SPI bus. CS management is critical — all four CS lines must default HIGH at power-on (firmware handles this via `spiQuiesceAll_()`).
2. **Heater control**: No SSR, relay, or PWM heater output on this board. The PID output is sent to ClearCore over TTL (`STAT;OUT=nnn`). ClearCore drives the actual heater hardware.
3. **3.3 V / 5 V interface**: ClearCore I/O is 3.3 V. Nano Every GPIO is 5 V. A 2-resistor voltage divider or level-shifter IC is recommended on the TTL RX line (ClearCore TX → XPB RX) to protect the ATmega4809 if it is ever powered while ClearCore is not.
4. **SD card init**: Firmware retries SD init up to 10 times with 100 ms backoff. Ensure SD socket VCC is stable before MCU boot completes if adding on-board power sequencing.
5. **LCD power-on**: LCD is initialized *after* SD to avoid LCD CS holding MISO low during SD init. PCB layout should not introduce parasitic paths between LCD and SD MISO.
6. **Form factor**: Nano Every module can be soldered via castellated pads or use through-hole pin headers (removable). Through-hole recommended for prototyping.
7. **Thermocouple connectors**: Position J_TC1 and J_TC2 on a panel edge accessible to the operator. K-type polarity must be observed on PCB footprint (positive = yellow wire on most US-standard K-type cables).
