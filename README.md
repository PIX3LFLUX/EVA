# Electric slot Vehicle Automation (EVA)
An autonomous slot car for Carrera Digital 124 / 132, project with hardware and software components.
It's  written with PlatformIO using the Arduino framework.

## General Information
This project was the author's Bachelor's Thesis at Hochschule Karlsruhe (HKA).
Documentation of the project as a whole is thus only available in german language.
The repository contents (including code comments) are english to ease adaption.

## Using the code
To run the software in any meaningful way, the hardware and a suitable development environment is needed.
For the software environment, Visual Studio Code can be used.
It needs the PlatformIO-Plugin as well as the firebeetle32 environment (though this should be downloaded and installed automatically on trying to compile).
For instrucions on how to develop using it, search for PlatformIO Visual Studio Code ESP32 - there are some very good tutorials on this topic online.

Once hardware and development environment are aquired, compile and flash the contents of the ```datalogger_sensorcar``` folder to the sensorcar.
Then, compile and flash the contents of ```controller_emulator_status_monitor``` folder to the controller emulator.
Set up a Raspberry Pi 4 according to the instructions in ```tools/pi-conf/instructions-to-setup-raspi```.
To modify behavior of the firmware, look at the ```globals.h``` file in the include folder of each project folder.
To understand code structure and functionality, start by looking at the main.cpp file, then work your way down as required.

The ```tools``` folder contains files that are used to assist development and are not required for functionaliy.

