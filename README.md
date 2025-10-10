# RTM

re-write of the Rotary Stand code base.

This project is for the controls for a rotary seal test stand. It can run various profiles with varying speeds, accelerations, dwells, temperatures, data collection
The idea is that the user can insert an SD card with the test profile, and the machine can upload and run it while saving data (if requested) to the SD card.

Currently the prototype board is connected via USB cable for firmware upload and terminal output via serial for debugging. This could be improved with a more robust testing protocol but we can evaluate that later.

hardware list & links to docs:
- motor controller etc. - clearcore hardware - https://teknic.com/files/downloads/clearcore_user_manual.pdf
- protocol/display/IO controller - arduino nano every
- SN74LVC245AN for shifting voltage to 3v3 for microSD card reader
- MAX31855 thermocouple inputs (1 on proto board, 2 on final build)
- newhaven LCD display - https://newhavendisplay.com/content/specs/NHD-0420D3Z-NSW-BBW-V3.pdf
- board comms using TTL between COM1 port on ClearCore and RX/TX pin on Nano Every. Eventually using a RS-485 transiever for long 10ft cable run between boards.
- the clearcore runs on 24V and will provide power to all other peripherals via 5V com port.
- The motor will be a ClearPath Motor, CPM-SDHP-N0563A-ELN - https://teknic.com/model-info/CPM-SDHP-N0563A-ELN_Fan/?model_voltage=230VAC3ph https://teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
- there will be two variants of the test stand the 2nd variant will have a 1:3 gearbox for achieving higher speeds.