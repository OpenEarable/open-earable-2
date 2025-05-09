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

## LED States
| Operational Mode         | LED Color & Pattern   | Meaning                                                                                                 |
|--------------------------|-----------------------|---------------------------------------------------------------------------------------------------------|
|                          |                       |                                                     |                                                    |
| **Charging**             |                       |                                                                                                         |
| – Power Connected        | Orange solid          | Power connected, charging current not yet verified or not at the configured level                      |
| – Bulk Charge            | Orange blinking       | At least 80% of the configured target charging current                                                  |
| – Trickle Charge         | Green blinking        | Float/constant-voltage charging reached; this state can be disabled via configuration                   |
| – Fully Charged          | Green solid           | Battery fully charged                                                                                   |
| – Pre-Charge/System Fault| Red blinking          | Pre-charge stage or system down; voltage not yet cleared                                                |
| – Battery Fault          | Red solid             | Any battery fault (e.g. deep discharge, current = 0)                                                     |
|                          |                       |                                                     |                                                    |
| **Discharging**          |                       |                                                                                                         |
| – Critical Battery Level | Red blinking (fast)   | Battery critically low (≈3% SOC or EDV1 reached)                                                         |
| – Low Battery Level      | Orange blinking (fast)| Battery low (≈7% SOC or EDV2 reached); state is disabled by default and must be enabled via configuration |




