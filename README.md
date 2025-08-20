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
   - Download and install from [https://code.visualstudio.com](https://code.visualstudio.com)

2. **Install the J‑Link Software and Documentation Package**
   - Download and install from [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/)
     
3. **Install nRF-Util**  
   - Download from [nRF Util – Nordic Semiconductor](https://www.nordicsemi.com/Products/Development-tools/nRF-Util)  
   - Add `nrfutil` to your system's `PATH` environment variable

4. **Install the nRF Connect for VS Code Extension**  
   - Open VS Code  
   - Go to the Extensions tab and install **"nRF Connect for VS Code"**  
   - Install all required dependencies when prompted

5. **Install the Toolchain via nRF Connect**  
   - Open the **nRF Connect** tab in VS Code  
   - Click **"Install Toolchain"**  
   - Select and install **version 3.0.1**

6. **Install the nRF Connect SDK**  
   - In the **nRF Connect** tab, select **"Manage SDK"**  
   - Install **SDK version 3.0.1**

7. **Open the Firmware Folder in VS Code**  
   - Use `File > Open Folder` or drag-and-drop the firmware directory into VS Code
   - OR in the **APPLICATIONS** section of the nRF Connect tab:
     - Select `Open Exisiting Application`
     - Select the `open-earable-2` directory

8. **Configure the Application Build**
   - If not already open, navigate to the nrfConnect extension tab in VSCode
   - In the **APPLICATIONS** section of the nRF Connect extension tab:  
     - Select the `open-earable-2` application  
     - Click **"+ Add build configuration"** to set up a new build
     - Select the SDK version 3.0.1, toolchain version 3.0.1, and `open-earable-2/nrf5340/cpuapp` as board target
     - To build **with FOTA** (firmware over-the-air update functionality):
       - Leave the `Base configuration files (Kconfig fragments)` dropdown empty
       - as `Extra CMAKE arguments` set `-DFILE_SUFFIX="fota"`
       - as `Build directory` name set `build_fota`
     -  To build **without FOTA**:
        - Select `prj.conf` as the `Base configuration files (Kconfig fragments)`
        - Do not set any of the FOTA flags described above
    
10. **J-Link Setup**
   - Wire your J-Link to the debugging breakout PCB as shown below.
     ![image](https://github.com/user-attachments/assets/2eeec41e-6be1-4a4f-b986-7d9a07b0f8e5)


11. **Build and Flash**
   - Click on `Generate and Build` and wait for the application to build (this will take some time)
   - Open a new terminal in VS Code and run the following command from the root of the `open-earable-v2` directory to flash the FOTA build. Make sure to set the serial number of your J-Link (right click your J-Link in the `CONNECTED DEVICES` tab of the nRF connect extension and copy the serial number).
     ```bash
     ./tools/flash/flash_fota.sh --snr 123456789 --left    # --right for the right ear device, or no flag to retain left/right bonding
     ```
   - or without FOTA
     ```bash
     ./tools/flash/flash.sh --snr 123456789 --left        # --right for the right ear device, or no flag to retain left/right bonding
     ```



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






