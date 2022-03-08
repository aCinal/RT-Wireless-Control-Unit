# Wireless Control Unit

Event-driven run-to-completion software of the *Wireless Control Unit* (WCU) used in *PWR Racing Team* Formula Student cars: RT11 and RT12e.

Use cases:
- Handle wireless sensors by interfacing with Bluetooth-enabled *Wireless Data Transfer System* (WDTS) via UART -> `Core/Src/wcu_bt.c`
- Implement real-time telemetry forwarding CAN bus traffic via a 2.4 GHz XBee Pro unit -> `Core/Src/wcu_xbee.c`
- Interface with an SD card, where runtime logs are being stored as well as "telemetry subscription" (filter mask) -> `Core/Src/wcu_sdio.c`
- Interface with a Quectel L26 GNSS module via UART -> `Core/Src/wcu_gnss.c`
- Interface via SPI with a 433 MHz S2-LP unit to receive data from a TPMS (proof-of-concept, not completed) -> `Core/Src/s2lp_api.c`

Project created and configured using STM32CubeIDE.
