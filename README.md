# SMuRF cryostat board PIC code

## Description

This is the original code for the SMuRF cryostat board.

The board is bases on the microchip's microcontroller PIC32MX795F512L.

The code was developed using Microchip's MPLAB X and the MPLAB Harmony Configuration (MHC) framework.

## Software package versions

The current version of the software tool used are:
- On macOS 10.15.6
  - MPLAB X: v5.40
  - HMC: v2.06
  - XC32: v2.41
- On Windows 10:
  - MPLAB X: v5.15
  - HMC: v2.06
  - XC32: v1.42

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
