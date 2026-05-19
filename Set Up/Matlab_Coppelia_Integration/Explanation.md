# Integration between Matlab and CoppeliaSim 
We are currently working on the API as the middleware between CoppeliaSim and Matlab to give commands and get info

## Scene Setup



### Pioneer Lua
In the Pioneer lua we have constructed a C++ in such a way that we can keep the robot static (do not move when we run the code from CoppeliaSim)


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
