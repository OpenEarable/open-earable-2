# OpenEarable 2 - Firmware

[OpenEarable](openearable.com) is the world's first fully open-source AI platform for ear-based sensing applications with true wireless audio. Packed with an unprecedented array of high-precision sensors, OpenEarable redefines what's possible in wearable tech. Designed for both development and research applications, OpenEarable is modular, reconfigurable, and built for the future.
<br/><br/><br/>
<img width="1323" height="568" alt="image" src="https://github.com/user-attachments/assets/ab4e2b0e-9645-4428-975f-466c7ef2f13b" />


## Table of Contents

1. [Setup](#setup)

2. [Battery States](#battery-states)

3. [Connection States](#connection-states)  

4. [SD Card](#sd-card)
   
5. [Citing](#citing)


**NOTE** - The OpenEarable 2.0 has a known issue where the flex cable to the sensors can come loose, causing the sensors to stop working. If this happens, you can try to fix it by reseating the flex cable.

## Setup
1. **Install Visual Studio Code (VS Code)**  
   - Download and install from [https://code.visualstudio.com](https://code.visualstudio.com).

2. **[OPTIONAL: JLINK FLASHING] Install the J‑Link Software and Documentation Package**
   - Download and install from [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/).
   - NOTE: install the version of JLink that is compatible with nRF Connect SDK 3.0.1 -- currently JLink 9.24a.
     
3. **Install nRF-Util**  
   - Download from [nRF Util – Nordic Semiconductor](https://www.nordicsemi.com/Products/Development-tools/nRF-Util).
   - Add `nrfutil` to your system's `PATH` environment variable.

4. **Install the nRF Connect for VS Code Extension**  
   - Open VS Code.
   - Go to the Extensions tab and install **"nRF Connect for VS Code"**.
   - Install all required dependencies when prompted.

5. **Install the Toolchain via nRF Connect**  
   - Open the **nRF Connect** tab in VS Code.
   - Click **"Install Toolchain"**.
   - Select and install **version 3.0.1**.

6. **Install the nRF Connect SDK**  
   - In the **nRF Connect** tab, select **"Manage SDK"**. 
   - Install **SDK version 3.0.1**.

7. **Open the Firmware Folder in VS Code**  
   - Use `File > Open Folder` or drag-and-drop the firmware directory into VS Code.
   - OR in the **APPLICATIONS** section of the nRF Connect tab:
     - Select `Open Exisiting Application`.
     - Select the `open-earable-2` directory.

8. **Configure the Application Build**
   - If not already open, navigate to the nrfConnect extension tab in VSCode.
   - In the **APPLICATIONS** section of the nRF Connect extension tab:  
     - Select the `open-earable-2` application.  
     - Click **"+ Add build configuration"** to set up a new build.
     - Select the SDK version 3.0.1, toolchain version 3.0.1, and `open-earable-2/nrf5340/cpuapp` as board target.
     - Leave the `Base configuration files (Kconfig fragments)` dropdown empty.
     - The default build includes FOTA (firmware over-the-air update) support.
    
9. **OPTIONAL: J-Link Setup**
   - Wire your J-Link to the debugging breakout PCB as shown below.
   ![image](https://github.com/user-attachments/assets/2eeec41e-6be1-4a4f-b986-7d9a07b0f8e5)
   - If you do not own a J-Link yet, here are a few options (do **NOT** use J-Link clones, they will not work and are illegal!):
      - [J-Link EDU Mini](https://mou.sr/3LrwiVe) (available to educational institutions, private persons, and students) with [JTAG adapter](https://www.adafruit.com/product/2094) and [cable](https://www.adafruit.com/product/1675).
      - Full-scale J-Link for commercial use (e.g., [J-Link BASE Compact](https://mou.sr/4oQkAls)).
      - ⚠️ The wiring show in the figure above is for the full-scale J-Link pinout. If you use the [JTAG adapter](https://www.adafruit.com/product/2094) the wiring may be different so make sure it is correct in your case!.

11. **Build and Flash**
   - Click on `Generate and Build` and wait for the application to build (this will take some time)
   - Make sure your device is charged or powered via USB. If the battery is fully discharged, the charging management IC will no longer supply power to the MCU from the battery, so you won’t be able to flash the MCU unless the battery is charged or the device is directly powered via USB.

   **[JLINK FLASHING]**
   - Open a new terminal in VS Code and run the following command from the root of the `open-earable-v2` directory to flash. Make sure to set the serial number of your J-Link (right click your J-Link in the `CONNECTED DEVICES` tab of the nRF connect extension and copy the serial number).
   ```bash
   # --right for the right ear device, or no flag to retain left/right bonding, --standalone for no pair   
   # --hw version is optional and can only be used with --left or --right
   ./tools/flash/flash.sh --snr 123456789 --left --hw 2.0.1    
   ```
     
   - The flash script is also available for Windows as `./tools/flash/flash.ps1`. To execute it, open PowerShell with administrative privileges.

   **[FOTA FLASHING via the Phone App]**
   - Charge to ~80% or more, unplug the earbud
   - Copy build/dfu_application.zip to your phone (either plug your phone is and transfer via USB, or upload it to Google Drive, or however)
   - Open the OpenEarable app on your phone
   - Connect to the device, and click the update button next to the firmware
   - Click the "+" button to add a custom firmware
   - Select the dfu_application.zip file
   - Click through to flash the firmware
   - (wait for the little red light *inside* the earbud to stop flashing)

   **[SERIAL FLASHING via USB]**
   - MCUboot is enabled by default. Install prerequisites:
   ```bash
   nrfutil install mcu-manager
   ```
   - Power off the device
   - Remove any SD card (the SD card conflicts with the USB serial port!)
   - Power on the device
   - Plug in the device via USB, verify that you see the serial port appear (for example /dev/ttyACM0)
   - Use the mcu-manager_upload.sh script, which uploads both app and network core images, marks them for testing, and resets the device:
   ```bash
   ./tools/flash/mcu-manager_upload.sh
   ```
   - You may need to edit the serial port in the script (default: `/dev/ttyACM0`).


11. **Recover Board**
   - If the application or network core becomes unresponsive, or you encounter flashing issues, you can recover the board using the recovery script. The `--snr` parameter specifies the serial number of your J-Link debugger.
   - Ensure the device is powered via USB or that the battery is sufficiently charged before running the recovery process. Otherwise, the MCU may not power up correctly and the recovery will fail.
   ```bash
   ./tools/flash/recover.sh --snr 123456789
   ```
   - After successful recovery, you can attempt to flash the firmware again.
   
12. **Enable Debug Output**
   - Open the **J-Link Configuration** program on your computer.  
      - On macOS: Press `CMD` + `Space` and search for `J-Link Config`.  
      - On Windows: Search for the program from the taskbar.  
   - Ensure your J-Link is connected to your computer.  
   - In the **Connected via USB** table, locate your J-Link device. Double-click it or right-click and select **Configure**.  
   - Find the **Virtual COM-Port** option and select **Enable**. Click **OK** to apply the setting.  
   - Open **Visual Studio Code**.  
   - In the left sidebar, open the **Extensions** menu.  
      - Search for and install the [**Serial Monitor**](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) extension.  
      - In the top menu bar, click **Terminal → New Terminal**.  
      - A terminal window will appear at the bottom of VS Code. Open the **Serial Monitor** tab.  
      - In the **Port** dropdown menu, select your J-Link’s COM port.  
      - Set the **Baud rate** to **115200**.  
      - Click **Start Monitoring**.  
      - Ensure your earable is connected to the debugger probe. You should now see debug output appearing when you interact with the device (e.g., press button).

13. **Recover Board**
   - If the application or network core becomes unresponsive, or you encounter flashing issues, you can recover the board using the recovery script. The `--snr` parameter specifies the serial number of your J-Link debugger.
   - Ensure the device is powered via USB or that the battery is sufficiently charged before running the recovery process. Otherwise, the MCU may not power up correctly and the recovery will fail.
      ```bash
      ./tools/flash/recover.sh --snr 123456789
      ```
   - After successful recovery, you can attempt to flash the firmware again (you will have to restore left/right bonding and hardware version).

## Mounting the SD Card over USB-C
Connect the device to USB-C, and then press the reset button to power-cycle the device. When it comes up, it should mount!

Note: The SD card is automatically unmounted from USB when the firmware starts logging (e.g. `sensor stress sd`), and re-mounted when logging stops.

## RTT Shell
To access the Zephyr shell over RTT (e.g. for running `sensor` commands), use J-Link with netcat:

**Terminal 1** — start J-Link:
```bash
JLinkExe -device nrf5340_xxaa_app -if swd -speed 4000 -autoconnect 1
```

**Terminal 2** — connect to RTT:
```bash
JLinkRTTClient -LocalEcho Off
```

Note: You must stop `JLinkExe` (Ctrl-C) before flashing!


## Error LED (Internal Red LED)
There is a separate red LED connected to the `#ERROR` pin (GPIO1:8) on the MDBT board, distinct from the main RGB status LED. It is defined as `led_error` in the devicetree.

| LED State         | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| Blinking          | MCUboot firmware upgrade in progress                                        |
| Solid on          | MCUboot found no bootable image, or the app hit a fatal error               |
| Off               | Normal operation (valid image booted successfully)                          |

## Battery States
Battery states will overwrite LED connection states. All LED states can be manually overwritten via BLE service.

### Charging States

| LED State         | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| 🟥 Red - Solid      | Battery fault or deep discharge*, charging current = 0                       |
| 🔴 Red - Pulsing    | Pre-charge phase or system-down voltage not yet cleared                     |
| 🟧 Orange - Solid   | Power connected, but charging current is not verified or not at desired level |
| 🟠 Orange - Pulsing | At least 80% of the target charging current is reached                      |
| 🟢 Green - Pulsing  | Trickle charge; final voltage (constant voltage) reached. Can be disabled via config |
| 🟩 Green - Solid    | Fully charged                                                               |

*If your OpenEarable goes into deep discharge (solid red) after pre-charge (red pulse), you can unplug the OpenEarable and plug it in again. This should recover the device.


### Discharging States

| LED State           | Description                                                              |
|--------------------|--------------------------------------------------------------------------|
| 🟠 Orange - Blinking | Battery low (7% remaining or EDV2 reached). Disabled by default, enable via config |
| 🔴 Red - Blinking      | Battery critical (3% remaining or EDV1 reached)                          |


## Connection States
Battery states will overwrite LED connection states. All LED states can be manually overwritten via BLE service.

| LED State                           | Description                                                                 |
|-------------------------------------|-----------------------------------------------------------------------------|
| 🔵 Blue – Blinking Very Fast        | Configured as **left device**, searching for **right device**               |
| 🔴 Red – Blinking Very Fast         | Configured as **right device**, searching for **left device**               |
| 🔵 Blue – Blinking Fast             | Paired with left/right, **ready for device bonding**                        |
| 🔵 Blue – Blinking Slow             | Bonded, **waiting for connection**                                          |
| 🟢 Green – Blinking Slow            | **Connected**                                                               |
| 🟣 Purple – Blinking Slow           | **SD card recording**                                                       |

## SD Card
Because ZephyrOS does not allow remounting of SD cards, it is **very important that the device is turned of before inserting or removing the SD card**.
As long as a recording to the SD card is active, the LED light will blink purple.


### File Parsing
Files recorded to the local microSD card in the binary `*.oe` format can be parsed using <a href="https://colab.research.google.com/drive/1qwdvjAM5Y5pLbNW5t3r9f0ITpAuxBKeq" target="_blank">this Python notebook</a>.

## Citing
If you are using OpenEarable, please cite is as follows:
```
@article{roddiger2025openearable,
     title = {OpenEarable 2.0: Open-Source Earphone Platform for Physiological Ear Sensing},
     author = {Röddiger, Tobias and Küttner, Michael and Lepold, Philipp and King, Tobias and Moschina, Dennis and Bagge, Oliver and Paradiso, Joseph A. and Clarke, Christopher and Beigl, Michael},
     year = 2025,
     journal = {Proceedings of the ACM on Interactive, Mobile, Wearable and Ubiquitous Technologies},
     volume = {9},
     number = {1},
     pages = {1--33},
     publisher={ACM New York, NY, USA}
}
```