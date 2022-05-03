# clamp_controller
Python-based high-level controller to monitor and command a network of distributed clamps.

This repo is part of the [Robotic Assembled Timber Structures with Integral Timber Joints](https://github.com/gramaziokohler/integral_timber_joints) project.

## Repo folder structure

**/src/clamp_controller** - Contains all the import-able class and functions.

**/src/controller_instances** - Contains instances of customized controllers for different projects.

**/src/serial_radio_transport_driver** - A light weight library for transporting Serial Message to the [USB Dongle](https://github.com/gramaziokohler/clamp_electronics/blob/master/00_USBRadioDongle/00_USBRadioDongle.md), the message format is described in the [radio firmware](https://github.com/gramaziokohler/clamp_firmware/tree/master/serial_radio).

## Design Goals

**ClampModel**

Digital twin of a clamp. Contains all the properties (e.g. step/mm conversion, soft limit, address) of a clamp and the telemetry reading.

Also contain functions to:

- Decode received telemetry

- Conversion between linear position and step position.

**SerialCommander**

The main model of the application. Contains:

- Instance of all available `ClampModel`
- Instance of the `SerialPort` that connects to the USBRadioDongle
- Instance of the `RosClampCommandListener` that connects to a ROS core via roslibpy

Functions to:

- Accept high-level synchronized move command for multiple clamps.
- Create low level move, home, speed-set and stop command.
- Send low level command to clamp with resend-on-Nack option.

**RosClampCommandListener**

A class that maintains the connection with ROS via roslibpy and listens-for (and replies-to) commands received as a rostopic.

**CommanderGUI**

A long function that create the tkinter UI for monitoring and control. This UI

Alternatively, it is possible to run the `SerialCommander` directly without UI, all the connections and settings can be accessed by code.

**RemoteClampFunctionCall**

A class for calling the clamp function from a remote process via roslibpy and ROS.

## Installation

1. Clone the [this repository](https://github.com/gramaziokohler/clamp_controller).
2. Change directory `cd` to the local location of this repository.
3. Activate your virtual environment (optional).
4. Install the packages in the /src folder and the dependencies:

```
pip install -r requirements-dev.txt
```

## Network Setup

Multiple devices need to cooperate with each other during robotic execution. A number of networks are present to connect the devices to the control computer (which is my Laptop) running the Process Execution Controller. Below is a typical network configuration for the devices on the network.

**Laptop (Ethernet Port)** - connection to RFL ethernet network (192.168.0.0/24 subnet)

- Linux Virtual Machine running [ROS Master, ROS Bridge and ROS RRC_Driver](https://compas-rrc.github.io/compas_rrc/latest/reference/index.html) (192.168.0.117)
- ABB Robot Controller running RRC
- Routable Internet via ETH guest network

**Laptop (Wireless Adapter)** - connection to ESP32 camera network (192.168.1.0/24 subnet)

- 4 (or more) ESP32 cameras (192.168.1.100 to 192.168.1.103)

**Laptop (USB COM Port)** - connection to USB Radio Dongle (proprietary network)

- 4 Clamps (address 1-4)
- 4 Screwdrivers (address 5-8)

(there are alternative network setup possible, for example using USB-Ethernet Adapter for connecting to the Wireless AP)

(the ROS instances can also be run from my Laptop's VMware Workstation Ubuntu 16 Virtual Machine bridged to the RFL network)

## Usage

### Controller with front end user interface:

1. To start the controller UI. Change directory to where you want the log files to be placed.
2. Run one of the controller UI instance, for example:

```
cd local_directory_for_log_files
python local_path_to_repo\src\controller_instances\08_TokyoCommander_UI+ROS.py
python -m controller_instances.10_Commander4Clamps4Screws
```

UI like this should appear:

![UI_Tokyo_JustStarted](doc/UI_Tokyo_JustStarted.jpg)

### Remote calling clamp functions

To remotely call clamp functions from a separate python execution/process (e.g. [itj_process](https://github.com/gramaziokohler/itj_process) controllers), you need to import **RemoteClampFunctionCall**. The following things should be started before making calls:

- ros and ros_bridge is started on the linux machine, for example: ` roslaunch rosbridge_server rosbridge_websocket.launch`
- the calling machine and the ros machine is on the same local network.
- a clamp_controller Controller Instance has started and is already connected to ROS

Example code:

```
# Connect to ROS via roslibpy
   hostip = '192.168.43.141'
   clamps_connection = RemoteClampFunctionCall(hostip)

# Command to send clamp to target (non-blocking)
   clamps_connection.sendD(ROS_VEL_GOTO_COMMAND)
# Command to send clamp to target (blocking)
   success = clamps_connection.send_and_wait(ROS_VEL_GOTO_COMMAND, 1000)
# Command to stop clamps (non-blocking)
self.
   clamps_connection.send(ROS_STOP_COMMAND(['1','2']))
```

Read the [RemoteClampFunctionCall.py](src\clamp_controller\RemoteClampFunctionCall.py) for available functions.



Addresses
-------------

In the latest use case, 4 Clamps and 4 Screwdrivers are used, also including 4 cameras on Clamps and 1 camera on Toolcanger. The addresses are as follows:

| Type       | id   | radio address | camera address |
| ---------- | ---- | ------------- | -------------- |
| CL3        | c1   | 1             | 192.168.1.101  |
| CL3        | c2   | 2             | 192.168.1.102  |
| CL3M       | c3   | 3             | 192.168.1.103  |
| CL3M       | c4   | 4             | 192.168.1.104  |
| SL1        | s1   | 5             |                |
| SL1        | s2   | 6             |                |
| SL1        | s3   | 7             |                |
| SL1_G200   | s4   | 8             |                |
| TC4_Camera |      |               | 192.168.1.100  |

Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> [@yck011522 ](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler)

