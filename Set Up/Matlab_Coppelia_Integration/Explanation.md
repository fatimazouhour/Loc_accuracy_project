# Integration between Matlab and CoppeliaSim 
We are currently working on the API as the middleware between CoppeliaSim and Matlab to give commands and get info

## Scene Setup



### Pioneer Lua
In the Pioneer lua we have constructed a C++ in such a way that we can keep the robot static (do not move when we run the code from CoppeliaSim).
In addition to that we have used those two lines:

    sim.setFloatSignal("leftEncoder", leftEncoder)
    sim.setFloatSignal("rightEncoder", rightEncoder)
in order to set the leftEncoder and rightEncoder at floating signals that will be recognizable for matlab through API, this os used as a substitute for the odometer sensor, because there is no odometer sensor in CoppeliaSim default files.


### GPS Lua
For GPS, you have to go to the components file in the 'Model browser' panel, and then to 'sensors' file and then drag the GPS sensor to the PioneerP3DX component in 'Scene hierarchy' panel. 
After you add the GPS, navigate to the GPS component inside the PioneerP3DX, and change the GPS lua C++ script to be the one attached here in the file.

In the lua of GPS you can see those lines:

    objectAbsolutePosition=sim.getObjectPosition(ref)
    
    --Now add some noise to make it more realistic:
    objectAbsolutePosition[1]=objectAbsolutePosition[1]+2*(math.random()-0.5)*xNoiseAmplitude+xShift
    objectAbsolutePosition[2]=objectAbsolutePosition[2]+2*(math.random()-0.5)*yNoiseAmplitude+yShift
    objectAbsolutePosition[3]=objectAbsolutePosition[3]+2*(math.random()-0.5)*zNoiseAmplitude+zShift
    
    --Setting those float values in order for matlab to recognize
    sim.setFloatSignal('gpsX', objectAbsolutePosition[1])
    sim.setFloatSignal('gpsY', objectAbsolutePosition[2])
    sim.setFloatSignal('gpsZ', objectAbsolutePosition[3])
In those lines, we are getting the true position of the robot from the simulation directly and then adding random variable for the position to represent the noise and uncertainty of a generic GPS.
After that we have also set the objectAbsolutePosition element as floating signals named gpsX, gpsY, gpsZ to be recognizable for matlab through API.
Other than setting the float signal lines, the lines are the default from CoppeliaSim.

### GyroSensor Lua
For Gyroscope, you have to go to the components file in the 'Model browser' panel, and then to 'sensors' file and then drag the Gyrosensor sensor to the PioneerP3DX component in 'Scene hierarchy' panel. 
After you add the GyroSensor, navigate to the GyroSensor component inside the PioneerP3DX, and change the GyroSensor lua C++ script to be the one attached here in the file.

In the script of GyroSensor you can see those lines:

    local euler=sim.getEulerAnglesFromMatrix(m)
    gyroData[1]=euler[1]/dt
    gyroData[2]=euler[2]/dt
    gyroData[3]=euler[3]/dt
    sim.setFloatSignal('roll', gyroData[1])
    sim.setFloatSignal('pitch', gyroData[2])
    sim.setFloatSignal('yaw', gyroData[3])

In this sequence of lines, we get the euler angles from the multiple of matrices between the angles and the transformationMatric and then we define the gyroData as the euler derivates. 
After than we set the roll, pitch, yaw angles from the gyroData as float signals to be recognizable by matlab through API.
Other than setting the float signal lines, the lines are the default from CoppeliaSim.

### Accelerometer Lua

For Accelerometer, you have to go to the components file in the 'Model browser' panel, and then to 'sensors' file and then drag the Accelerometer sensor to the PioneerP3DX component in 'Scene hierarchy' panel. 
After you add the Accelerometer, navigate to the Accelerometer component inside the PioneerP3DX, and change the Accelerometer lua C++ script to be the one attached here in the file.

In the script of Accelerometer you can see those lines:

        accel={force[1]/mass,force[2]/mass,force[3]/mass}
        sim.setFloatSignal('accelX', accel[1])
        sim.setFloatSignal('accelY', accel[2])
        sim.setFloatSignal('accelZ', accel[3])

In those lines, the data coming from the Accelerometer sensor is the division between the forces on the three axes given by the simulation directly, and then divided by the mass, resulting at the end the acceleration.
After calculating the accelX = accele[1], accelY = accel[2], accelZ = accel[3], now we set the values as floating signals to be recognizable by matlab through API.
Other than setting the float signal lines, the lines are the default from CoppeliaSim.

