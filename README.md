# Welcome to UrdfSim

UrdfSim is a fork of the [AirSim simulator](https://github.com/Microsoft/AirSim) that attempts to solve the problem of simulation of arbitrary robots. This fork maintains backwards compatibility with the existing AirSim features, while adding the capability of simulation of arbitrary robot, with control via python and c++ APIs. This simulator is built upon the excellent AirSim simulator released by MSR, and the Unreal Engine. 

This code base works with UE 4.24. Other verisons may work, but are not explicitly tested or supported.

Here are some examples of bots that can be built inside of AirSim-URDF:

![LunabotPicture](docs/images/Lunabot.gif)
![ArmPicture](docs/images/Arm.gif)

## How to Get It

### Windows
* [Build it](docs/build_windows.md)

### Linux
* [Build it](docs/build_linux.md)

To get started with the custom robot functionality, proceed to [Using a URDF pawn](docs/UsingAUrdfPawn.md).