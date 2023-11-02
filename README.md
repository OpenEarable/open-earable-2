# open-earable-v2

## Development
### Setting Up Your IDE

#### Installing nRF Connect SDK and VS Code[^1]
1. **Install nRF Command Line Tools**
Download the latest version that matches your operating system.
https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download?lang=en#infotabs 
<br>*Ubuntu and macOS:*
J-Link Software and Documentation pack must be downloaded and installed manually from https://www.segger.com/downloads/jlink/. On Windows, this is managed through the nRF Command Line Tools itself.

2. **Install VS Code**
Go to https://code.visualstudio.com/download and install the version that matches your operating system.

3. **Install nRF Connect Extension Pack**
In VS Code go to the Extensions tab, then type "nRF Connect for VS Code Extension Pack" in the search field, and click on Install.

4. **Install Toolchain**
The first time you open nRF Connect for VS Code, it will prompt you to install a toolchain. Click on "Install Toolchain".

5. **Install nRF Connect SDK**
In the nRF Connect for VS Code tab, click on Manage SDK, and Install SDK. It will list the available versions of the nRF Connect SDK that can be downloaded and installed on your machine. Choose the nRF Connect SDK version you wish to use for your project development. Please note that the use of main is not encouraged (not thoroughly tested & no technical support)

With this, we have completed the installation of nRF Connect SDK and VS Code. Now, we can build and flash our nRF Connect SDK application to our board.



[^1]: https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/