

# ReactionWheelsHandler Documentation

This documention gives a short overview over the ReactionWheelsHandler - a reaction wheels device handler within the Flight Software Framework.

## Development Environment
The development environment covers soft- and hardware that was used in the development process of the ReactionWheelsDeviceHandler.

### Hardware
As access to real life Reaction Wheels is rather difficult, it was decided to simulate a reaction wheel on an Arduino Nano 33 Sense Rev2. It has one programmable LED that is used to show the rotation speed of the reaction wheel via blinking accordingly.
To communicate with the Arduino, it is then connected via USB to the computer on which the FSFW runs.

### Software
Softwarewise, Archlinux is used as the operating system on which the Flight Software Framework is run. Archlinux in turn runs as a WSL on a Windows computer.
As the IDE to develop the code Visual Studio code was used, which can directly connect to the WSL.

## Prerequisites and Preparations

To run the software successfully, your **ArchLinux WSL** should have installed the following software:

 - Python 3 with pip and pyserial
 - CMake
 - Git
 - nano
 -  zsh

For "Quality-of-Life" it is recommended to have also installed usbutils for debugging the connection of the Arduino to the WSL.

To properly connect the Arduino to the WSL, **usbipd** needs to be installed. We use this software in a script (USBtoWSL.ps1) to automate the connecting process. Within the script the BUSID 1-6 is set as the Arduino's BUSID. Check via usbipd, if this is correct for your system as well, then run the script while the WSL is running and the Arduino is connected to your system. The Arduino will then automatically connect to your WSL.

## Installation

To now use the software on your WSL, clone the repository to your WSL.

Clone it from this repository:
https://github.com/JoestHomann/fsfw

After cloning, make a new directory for building, then build the FSFW within this folder via the following commands:

 - cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DOS_FSFW=linux ..
 - cmake --build . -j

Then run it: 

 - ./fsfw-example-hosted

The FSFW should now start without error messages and we can start commanding it.

## Commanding

To send commands to the ReactionWheelsHandler and therefore the Arduino (reaction wheel),
run the rw_pus.py script (can be found under: /ReactionWheelSim/rw_pus.py).

For the ReactionWheelsHandler these commands are implemented:

 - mode \<on/normal/off>
	 - \<on>: Device handler transitions to mode on, only status messages will be forwarded to the Arduino. No status polling, takes place when in mode on.
	 - \<normal>: Device handler will forward all valid incoming commands (speed, torque, status) and regularly poll status to update the local data pool (RwPusService forwards values to rw_pus.py)
	 - \<off>: Stops reaction wheel (0 RPM) and wont accept any commands
 - speed \<value>: Sends a target speed to the Arduino
 - torque \<value>: Sends a target torque to the Arduino
 - stop: Arduino decelerates to 0 RPM
 - status: Requests status with current speed, torque, etc. and Arduino answers with the current values

To further show how the ReactionWheelHandler interacts with other devices in the framework, a AcsController was implemented. It can be commanded as following:

 - acs_enable \<1/0>: 
	 - \<1>: Enables the AcsController, regularly updates the attitude local data pool values 		(RwPusService forwards this to rw_pus.py via PUS-Service 220-133). It also sends a single messages with Information about the controller (PUS-Service 220-133)
	 - \<0>: Disable the AcsController, stops 
 - att \<q1> \<q2> \<q3> \<q4>: Sends the desired attitude to the controller (in quaternions)

The ACS is not part of the assignment, and therefore is - as already mentioned - only supposed to be used for show casing the behaviour of the ReactionWheelsHandler.


## Data flow
A simplified data flow of the communication between the ground commander via PUS-220 TC and the FSFW is shown below. It focuses on only showing the data flow to and from the ReactionWheelSim on the Arduino.

<img width="1447" height="530" alt="image" src="https://github.com/user-attachments/assets/34734b74-60b3-4dea-8092-f63cc0c252e3" />






