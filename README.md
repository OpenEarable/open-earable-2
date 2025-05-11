# OpenEarable 2 - Firmware

[OpenEarable](openearable.com) is the world's first fully open-source AI platform for ear-based sensing applications with true wireless audio. Packed with an unprecedented array of high-precision sensors, OpenEarable redefines what's possible in wearable tech. Designed for both development and research applications, OpenEarable is modular, reconfigurable, and built for the future.
<br/><br/><br/>
![image](https://github.com/user-attachments/assets/8cb55571-c6bc-4f51-b2ae-628f7be3661c)

## Table of Contents

1. [Setup](#setup)  
   1.1 [Install Visual Studio Code (VS Code)](#install-visual-studio-code-vs-code)  
   1.2 [Install nRF-Util](#install-nrf-util)  
   1.3 [Install the nRF Connect for VS Code Extension](#install-the-nrf-connect-for-vs-code-extension)  
   1.4 [Install the Toolchain via nRF Connect](#install-the-toolchain-via-nrf-connect)  
   1.5 [Install the nRF Connect SDK](#install-the-nrf-connect-sdk)  
   1.6 [Open the Firmware Folder in VS Code](#open-the-firmware-folder-in-vs-code)  
   1.7 [Configure the Application Build](#configure-the-application-build)  
   1.8 [Build and Flash](#build-and-flash)  

2. [Battery](#battery)  
   2.1 [Charging States](#charging-states)  
   2.2 [Discharging States](#discharging-states)  

3. [Connection States](#connection-states)  

4. [File Parsing](#file-parsing)  


## Setup
1. **Install Visual Studio Code (VS Code)**  
   Download and install from [https://code.visualstudio.com](https://code.visualstudio.com)

2. **Install nRF-Util**  
   - Download from [nRF Util – Nordic Semiconductor](https://www.nordicsemi.com/Products/Development-tools/nRF-Util)  
   - Add `nrfutil` to your system's `PATH` environment variable

3. **Install the nRF Connect for VS Code Extension**  
   - Open VS Code  
   - Go to the Extensions tab and install **"nRF Connect for VS Code"**  
   - Install all required dependencies when prompted

4. **Install the Toolchain via nRF Connect**  
   - Open the **nRF Connect** tab in VS Code  
   - Click **"Install Toolchain"**  
   - Select and install **version 3.0.1**

5. **Install the nRF Connect SDK**  
   - In the **nRF Connect** tab, select **"Manage SDK"**  
   - Install **SDK version 3.0.1**

6. **Open the Firmware Folder in VS Code**  
   - Use `File > Open Folder` or drag-and-drop the firmware directory into VS Code
   - In the **APPLICATIONS** section of the nRF Connect tab:
     - Select `Open Exisiting Application`
     - Select the `openearable-v2` directory

7. **Configure the Application Build**  
   - In the **APPLICATIONS** section of the nRF Connect tab:  
     - Select the `openearable-v2` application  
     - Click **"+ Add build configuration"** to set up a new build
   - Select the SDK version 3.0.1, toolchain version 3.0.1, and `openearable_v2/nrf5340/cpuapp` as board target
   - To build **with FOTA** (firmware over-the-air update functionality):
     - TODO: proj. conf settings
     - as `Extra CMAKE arguments` set `-DFILE_SUFFIX="fota"`
     - as `Build directory` name set `build_fota`
   -  To build **without FOTA**:
      - remove the FOTA flags from above
      - TODO: select the correct proj.conf

8. **Build and Flash**
   - Click on `Generate and Build` and wait for the application to build (this will take some time)
   - Open a new terminal in VS Code and run the following command from the root of the `open-earable-v2` directory to flash the FOTA build. Make sure to set the serial number of your JLink (right click your JLink in the `CONNECTED DEVICES` tab of the nRF connect extension and copy the serial number).
     ```
     ./tools/flash/flash_fota.sh -snr 123456789 --left
     ```
   - or without FOTA
     ```
     ./tools/flash/flash.sh -snr 123456789 --left
     ```



## Battery
### Charging States

| LED State         | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| 🟥 Solid Red      | Battery fault or deep discharge, charging current = 0                       |
| 🔴 Pulsing Red    | Pre-charge phase or system-down voltage not yet cleared                     |
| 🟧 Solid Orange   | Power connected, but charging current is not verified or not at desired level |
| 🟠 Pulsing Orange | At least 80% of the target charging current is reached                      |
| 🟢 Pulsing Green  | Trickle charge; final voltage (constant voltage) reached. Can be disabled via config |
| 🟩 Solid Green    | Fully charged                                                               |



### Discharging States

| LED State           | Description                                                              |
|--------------------|--------------------------------------------------------------------------|
| 🟠 Blinking Orange  | Battery low (7% remaining or EDV2 reached). Disabled by default, enable via config |
| 🔴 Blinking Red     | Battery critical (3% remaining or EDV1 reached)                          |


## Connection States
TODO

## File Parsing
Files recorded to the local microSD card in the binary `*.oe` format can be parsed using <a href="https://colab.research.google.com/drive/1qwdvjAM5Y5pLbNW5t3r9f0ITpAuxBKeq" target="_blank">this Python notebook</a>.






