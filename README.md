# Electric slot Vehicle Automation (EVA)
An autonomous slot car for Carrera Digital 124 / 132, project with hardware and software components.
It's  written with PlatformIO using the Arduino framework.

## The hardware
![EVA hardware](https://github.com/PIX3LFLUX/EVA/blob/master/eva-hardware.jpg?raw=true)

## General Information
This project was the author's Bachelor's Thesis at Hochschule Karlsruhe (HKA).
Documentation of the project as a whole is thus only available in german language.
The repository contents (including code comments) are english to ease adaption.

## Getting Started
### Prerequisites
- Python  
```
https://www.python.org/downloads/
```
Tick the "add to PATH" option just in case.
- Visual Studio Code  
```
https://code.visualstudio.com/Download
```
- PlatformIO IDE  
Open VSCode, then find the Extensions Marketplace on the left hand side or press Ctrl+Shift+X.
Search for "PlatformIO".
Once found, install it by clicking on "install".
- C/C++ Extension Pack  
Install it in the same way as PlatformIO IDE.

*Optional*
- Matlab  
```
https://www.mathworks.com/products/get-matlab.html
```
The project includes several Matlab scripts to aid development.
If Matlab is not available to You, try adjusting the syntax to use Octave instead.
```
https://www.gnu.org/software/octave/download
```

### Installation
- Download the code from this repository as a zip file.
- Unpack the archive into your working directory.
- Open VSCode.
- In VSCode, navigate to File > Open Folder or press Ctrl + K followed by Ctrl + O.
- Choose either "controller_emulator_status_monitor" or "datalogger_sensorcar" from this repository to open the respective project.
- In the EXPLORER on the left hand side, open the .vscode directory and click on c_cpp_properties.json. This will update the file to represent the paths on your development machine.
- On the bottom panel, click on the checkmark to build the project.
- Connect the ESP32 microcontroller to the development PC via USB.
- On the bottom panel, press the arrow pointing to the right to upload the code to the microcontroller.
- The code from "datalogger_sensorcar" is flashed to the vehicle, the code from "controller_emulator_status_monitor" is flashed to the stationary part of the system.
- To get a serial interface to the microcontroller for debugging, press the plug symbol on the bottom panel.
- Set up a Raspberry Pi 4 according to the instructions in "tools/pi-conf/instructions-to-setup-raspi" if output to a 1080p HDMI display is needed.  

*Optional*
- In VSCode, navigate to File > Preferences > Settings or press Ctrl+,.
- Search for "word wrap", then change "Editor: Word Wrap" from "off" to "on". The source code is designed to be viewed with word wrap.

## Usage
- You can find "globals.h" in the "include" folder using the EXPLORER on the left hand side.
- In both projects, this file is used to change behavior of the whole program easily.
- A description for every parameter is given as a comment in the file.
- State constants such as "SENSORCAR_IDLE_STATE" are not intended to be changed, as they are merely macros to make the code more readable.
- Extern variables that are declared in "globals.h" are initialized in "globals.cpp".
- The "tools" folder contains files that are used to assist development and are not required for functionaliy.

## Contributing
If you have a suggestion that would make this better, please fork the repository.
The author has finished development and will thus not be able to accept and process pull or feature requests.

## License
Distributed under the MIT License. See "LICENSE.txt" for more information.
