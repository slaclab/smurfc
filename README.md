# SMuRF cryostat board PIC code

## Description

This is the original code for the SMuRF cryostat board.

The board is bases on the microchip's microcontroller PIC32MX795F512L.

The code was developed using Microchip's MPLAB X and the MPLAB Harmony Configuration (MHC) framework.

## Software package versions

The current version of the software tool used are:
- MPLAB X: v5.15
- HMC: v2.06
- XC32: v1.42 on Windows 10 and v2.15 on macOS 10.13.6

## How to clone this code

This repository must be cloned in the HMC's `apps` directory which is locate at:

- on Windows:
```
C:\microchip\harmony\v2_06\apps\
```

- on macOS:

```
/Users/<YOUR_USER_NAME>/microchip/harmony/v2_06/apps
```

## SPI bus register map

The register map is describe in [README.SpiRegisterMap.md](README.SpiRegisterMap.md).