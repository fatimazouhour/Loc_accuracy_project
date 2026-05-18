sim = require 'sim'

function sysCall_init()
    motorLeft = sim.getObject("../leftMotor")
    motorRight = sim.getObject("../rightMotor")

    prevLeftPos = sim.getJointPosition(motorLeft)
    prevRightPos = sim.getJointPosition(motorRight)

    leftEncoder = 0
    rightEncoder = 0

    -- Force robot stopped at simulation start
    sim.setJointTargetVelocity(motorLeft, 0)
    sim.setJointTargetVelocity(motorRight, 0)
end

function sysCall_actuation()
    -- Do nothing here.
    -- MATLAB will control the wheel velocities.
end

function sysCall_sensing()
    local leftPos = sim.getJointPosition(motorLeft)
    local rightPos = sim.getJointPosition(motorRight)

    local dLeft = leftPos - prevLeftPos
    local dRight = rightPos - prevRightPos

    leftEncoder = leftEncoder + dLeft
    rightEncoder = rightEncoder + dRight

    prevLeftPos = leftPos
    prevRightPos = rightPos

    -- Send encoder values to MATLAB
    sim.setFloatSignal("leftEncoder", leftEncoder)
    sim.setFloatSignal("rightEncoder", rightEncoder)

    print("Left wheel encoder:", leftEncoder)
    print("Right wheel encoder:", rightEncoder)
end

function sysCall_cleanup()
    sim.clearFloatSignal("leftEncoder")
    sim.clearFloatSignal("rightEncoder")
end
