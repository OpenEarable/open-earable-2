# OpenEarable 2 - Firmware

[OpenEarable](openearable.com) is the world's first fully open-source AI platform for ear-based sensing applications with true wireless audio. Packed with an unprecedented array of high-precision sensors, OpenEarable redefines what's possible in wearable tech. Designed for both development and research applications, OpenEarable is modular, reconfigurable, and built for the future.
<br/><br/><br/>
![image](https://github.com/user-attachments/assets/8cb55571-c6bc-4f51-b2ae-628f7be3661c)

## Table of Contents

1. [Setup](#setup)

2. [Battery States](#battery-states)

3. [Connection States](#connection-states)  

4. [SD Card](#sd-card)
   
5. [Citing](#citing)


## Setup
1. **Install Visual Studio Code (VS Code)**  
   - Download and install from [https://code.visualstudio.com](https://code.visualstudio.com).

2. **Install the J‑Link Software and Documentation Package**
   - Download and install from [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/).
     
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
     - To build **with FOTA** (firmware over-the-air update functionality):
       - Leave the `Base configuration files (Kconfig fragments)` dropdown empty.
       - as `Extra CMAKE arguments` set `-DFILE_SUFFIX="fota"`.
       - as `Build directory` name set `build_fota`.
     -  To build **without FOTA**:
        - Select `prj.conf` as the `Base configuration files (Kconfig fragments)`.
        - Do not set any of the FOTA flags described above.
    
9. **J-Link Setup**
   - Wire your J-Link to the debugging breakout PCB as shown below.
   ![image](https://github.com/user-attachments/assets/2eeec41e-6be1-4a4f-b986-7d9a07b0f8e5)
   - If you do not own a J-Link yet, here are a few options (do **NOT** use J-Link clones, they will not work and are illegal!):
      - [J-Link EDU Mini](https://mou.sr/3LrwiVe) (available to educational institutions, private persons, and students) with [JTAG adapter](https://www.adafruit.com/product/2094) and [cable](https://www.adafruit.com/product/1675).
      - Full-scale J-Link for commercial use (e.g., [J-Link BASE Compact](https://mou.sr/4oQkAls)).
      - ⚠️ The wiring show in the figure above is for the full-scale J-Link pinout. If you use the [JTAG adapter](https://www.adafruit.com/product/2094) the wiring may be different so make sure it is correct in your case! _(to be confirmed, picture coming soon)_.

10. **Build and Flash**
     - Click on `Generate and Build` and wait for the application to build (this will take some time).
     - Make sure your device is charged or powered via USB. If the battery is fully discharged, the charging management IC will no longer supply power to the MCU from the battery, so you won’t be able to flash the MCU unless the battery is charged or the device is directly powered via USB.
     - Open a new terminal in VS Code and run the following command from the root of the `open-earable-v2` directory to flash the FOTA build. Make sure to set the serial number of your J-Link (right click your J-Link in the `CONNECTED DEVICES` tab of the nRF connect extension and copy the serial number).

<<<<<<< HEAD
11. **Build and Flash**
   - Click on `Generate and Build` and wait for the application to build (this will take some time)
   - Open a new terminal in VS Code and run the following command from the root of the `open-earable-v2` directory to flash the FOTA build. Make sure to set the serial number of your J-Link (right click your J-Link in the `CONNECTED DEVICES` tab of the nRF connect extension and copy the serial number).
     ```bash
     # --right for the right ear device, or no flag to retain left/right bonding, --standalone for no pair
     # --hw version is optional
     ./tools/flash/flash_fota.sh --snr 123456789 --left --hw 2.0.1    
   
     ```
   - or without FOTA
     ```bash
     # --right for the right ear device, or no flag to retain left/right bonding
      # --hw version is optional
     ./tools/flash/flash.sh --snr 123456789 --left --hw 2.0.1      
     ```
=======
      ```bash
      # --right for the right ear device, or no flag to retain left/right bonding
      # --standalone for no pair

      ./tools/flash/flash_fota.sh --snr 123456789 --left 
      ```

     - or without FOTA

      ```bash
      # --right for the right ear device, or no flag to retain left/right bonding
      # --standalone for no pair

      ./tools/flash/flash.sh --snr 123456789 --left    
      ```
     - The FOTA update script is also available for Windows as `./tools/flash/flash_fota.ps1`. To execute it, open PowerShell with administrative privileges.
>>>>>>> 2cc8b14 (Update README with debug output instructions)

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






