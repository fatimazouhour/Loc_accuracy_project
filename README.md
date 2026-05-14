# Loc_accuracy_project
Robotics course project based on Sofia Yousuf's Localization Accuracy of Mobile robots in indoor to outdoor applications
connection is used with API's , both Coppelia Sim and matlab are working on WINDOWS os.

## Connecting MATLAB to CoppeliaSim (Windows)

This project uses MATLAB to control a Pioneer P3-DX robot simulated in CoppeliaSim via the **Legacy Remote API**.

### Setup

1. **Install CoppeliaSim EDU** for Windows from [coppeliarobotics.com](https://www.coppeliarobotics.com/downloads).

2. **Copy the Remote API files** into your MATLAB working folder:
   - `remApi.m` and `remoteApiProto.m` from  
     `CoppeliaSim_Edu\programming\legacyRemoteApi\remoteApiBindings\matlab\matlab\`
   - `remoteApi.dll` from  
     `CoppeliaSim_Edu\programming\legacyRemoteApi\remoteApiBindings\lib\lib\Windows\64Bit\`

3. **Open the port inside CoppeliaSim.** Add a non-threaded child script to the scene and put this in `sysCall_init()`:
```lua
   simRemoteApi.start(19999)
```
   Save the scene.

### Running

1. Open CoppeliaSim and load the scene.
2. **Press the blue Play arrow ▶** — the remote API port only listens while the simulation is running.
3. In MATLAB, run your script. The connection is established with:
```matlab
   vrep = remApi('remoteApi');
   vrep.simxFinish(-1);
   clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
```
   A returned `clientID > -1` means the connection succeeded.

### Notes

- MATLAB and CoppeliaSim run on the same Windows machine; `127.0.0.1` is localhost.
- The port number in the Lua script (`19999`) must match the one in `simxStart`.
- If MATLAB can't connect, check that the simulation is actively playing in CoppeliaSim.
