# SDHC

SDHC/SDXC card driver for embedded systems using the HSMCI interface.

The module provides SD card initialization and identification, CID/CSD and SD Status parsing, 512-byte sector read/write operations, bus-width and high-speed configuration, synchronization and TRIM/erase support. It is intended primarily as a higher-level SD card layer above the low-level HSMCI driver and can be used as a backend for filesystems such as FatFS.

## Features

- SDHC and SDXC card initialization
- 512-byte logical sector access
- Multi-block read and write
- CID, CSD, SCR and SD Status handling
- 1-bit / 4-bit bus configuration
- High-speed mode support
- Card geometry and identification information
- Allocation Unit size reporting
- TRIM / DISCARD support
- Erase timeout calculation

## Requirements

- FreeRTOS
- HSMCI low-level SD driver
- 32-bit aligned data buffers are recommended for DMA/PDC transfers

## Author

Jan Rusnak  
Copyright (c) 2025 Jan Rusnak

## License

ISC-style license. See the license header in the source files.
