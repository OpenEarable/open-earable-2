# OpenEarable 2 Firmware

## Setup
- Install Visual Studio Code
- Install nRF-Util and add to your PATH https://www.nordicsemi.com/Products/Development-tools/nRF-Util
- Install the nRF Connect for VS Code extension and all of its dependencies
- Open the nRF Connect tab in VC Code and "Install Toolchain", Install version 3.0.1
- In the nRF Connect tab in VS Code select "Manage SDK" and install SDK version 3.0.1
- open the firmware folder in vscode (e.g., drop in vscode)
- in the application section in the nrfconnect tool select the openearable-v2 application, click + Add build configuration
- <img width="761" alt="image" src="https://github.com/user-attachments/assets/f03680b2-28b0-417b-b4c5-628c7e2c92cf" />
- <img width="850" alt="image" src="https://github.com/user-attachments/assets/2dc1601d-bfa4-4afb-8814-63cfa6136f21" />
- <img width="808" alt="image" src="https://github.com/user-attachments/assets/a049faa4-b108-49af-8f3d-789b0bf166fb" />
- Click "Generate and Build" (this will take a while)
- <img width="938" alt="image" src="https://github.com/user-attachments/assets/fa99f1b9-9187-4f29-855b-5147269ed807" />


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

## File Parsing
Files recorded to the local microSD card in the binary `*.oe` format can be parsed using [this Python notebook](https://colab.research.google.com/drive/1qwdvjAM5Y5pLbNW5t3r9f0ITpAuxBKeq).






