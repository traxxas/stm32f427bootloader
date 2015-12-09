# PX4FMU Bootloader

## Introduction
This project is an alternative to the PX4 Bootloader. This version of the bootloader allows the STM32F427 (PX4FMU) to be upgraded from the microSD drive. This version of the STM32F427 bootloader does not currently support upgrade via the USB OTG interface.

## Description
The bootloader is installed in the first 16k FLASH memory segment on the STM32F427. The remaining FLASH segments are reserved for the Application.

The bootloader checks for the existance of a special file "UPGRADE.bin" on the microSD card.  If the file exists, the contents of the file are installed in the Application FLASH space beginning at the second FLASH memory segment address [0x08004000].

The intention of this bootloader is to allow the application to initiate its own upgrade.  The application will determine that an upgrade is required and creating the UPGRADE.bin file with the appropriate upgrade firmware.

## System Requirements
This project was created using Rowley CrossWorks for ARM Release 3.5.1

## Libraries
The bootloader project makes use of the following open source libraries:

* FatFS - FAT FileSystem drivers created by ELM (Electronic Lives Mfg) [http://elm-chan.org/fsw/ff/00index_e.html](http://elm-chan.org/fsw/ff/00index_e.html)
* STM32 Standard Peripheral Library - Standard Peripheral Library for the STM32F4xx
* STM32 Cortex Microcontroller Software Interface Standard (CMSIS) Library