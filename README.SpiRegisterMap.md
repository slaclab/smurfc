# SPI bus register map

## Description

This file describe the register map accessible trough the SPI bus.

## SPI protocol

Each command send through the SPI bus must contain a 32-bit payload, with the following format:

| 1 bit     | 11 bits | 20 bits
|-----------|---------|--------
| read flag | address | data

The MSB bit is the `read` flag. When this flag is set, indicates that the command is a read request. Otherwise it indicates that the command is a write request.

A write command will write the `data` field into the register pointed by the `address` field.

For a read command, the `data` field is ignored. To get the content of the register pointed by the `address` filed, a second identical read command must be send. The return packet's `data` filed will contain the requested data.

## Register map

| Address | Mode       | Description
|---------|------------|-----------------
| 0x00    | Read-only  | Firmware version, coded in HEX in three bytes, one byte per version digit.<br>For example: Version `R2.3.1` is coded as `0x020301`.
| 0x01    | Read-only  | Status register. Not implemented, it always returns 0.
| 0x02    | Read/Write | TES bias relay control. 12 bits, one bit per each relay., plus the main AC/DC relay in bit 12
| 0x03    | Read-only  | HEMT bias value.
| 0x04    | Read-only  | 50K bias value.
| 0x05    | Read-only  | Temperature value.
| 0x06    | Read-only  | Cycle counts. Return the number of SPI read operations.
| 0x07    | Read/Write | Power supply enables, 4 bits:<ul><li>bit 0 : HEMT</li><li>bit 1 : 50K</li>
<li>bit 2 : HEMT2</li><li>bit 3 : 50K2</li>
</ul>
| 0x08    | Read/Write | Flux ramp controls, not used!
| 0x09    | Read-only  | ID voltage value.
| 0x0A    | Read-only  | HEMT2 bias value.
| 0x0B    | Read-only  | 50K2 bias value.
| 0x0C    | Read-only  | ID2 voltage value.
