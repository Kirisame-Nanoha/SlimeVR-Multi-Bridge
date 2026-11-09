# SlimeVR-Multi-Bridge (English Translation Reference)
*Note: This document is provided for translation and reference purposes only.*

Mocopi SlimeVR Bridge App for Windows based on MoSlime

Since Bluepy is only available on Linux, this tool was created using Bleak to enable a direct connection with Mocopi on Windows.

## Disclaimer
This third-party software was created with the help of ChatGPT. Although basic functionality has been verified, please use it entirely at your own risk. Please note that the author takes absolute no responsibility for any disadvantages or damages incurred by using this software.
Mocopi is a trademark owned by Sony Corporation. Neither MoSlime (the base software) nor this tool is endorsed or approved by Sony Corporation or SlimeVR.
While this software does not modify the Mocopi hardware or official firmware itself, by using this software, you agree to accept all associated risks.

## Setup & Requirements
This project was developed using Python 3.11 in VS Code.
Please activate your virtual environment and install Bleak:
 .\Scripts\Activate.ps1
pip install bleak

If you want to compile the script into an executable (.exe), install the following:
pip install pyinstaller
pip install winrt
