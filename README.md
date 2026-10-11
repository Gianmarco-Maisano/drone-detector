DroneDetect is an external application for PortaPack Mayhem. 

The application passively scans configurable sections of the 2.4 GHz and 5.8 GHz bands for signals that may be associated with drone links. It does not decode any radio protocol, but it helps increase awareness of spectrum signals on frequencies used by commercial drones, with the option to set alert thresholds and trigger an audio beep whenever a threshold, based on both signal power and signal type, is exceeded.

## Repository Contents

The source files belong in the Mayhem external-app tree:

```text
firmware/application/external/dronedetect/
	main.cpp
	ui_dronedetect.cpp
	ui_dronedetect.hpp
	README.md
```

The application must also be registered by the Mayhem build system in `external.cmake` and `external.ld`. 

## Install from the SD Card

1. Back up the SD card.
2. Extract the release archive at the SD card root, merging its `APPS` and `FIRMWARE` directories with any existing directories.
3. Start the currently installed Mayhem firmware and open **Flash Utility** in  utilities.
4. Select `/FIRMWARE/portapack-mayhem-firmware.bin`, confirm the update, and allow the device to finish flashing and reboot. Do not power off during the update.
5. After booting the matching firmware, open the **RX** app menu and launch **DroneDetect**.

Mayhem loads .ppma apps from /APPS. Installing this app without a compatible firmware version may allow it to show up in the menu, but it won't function.

