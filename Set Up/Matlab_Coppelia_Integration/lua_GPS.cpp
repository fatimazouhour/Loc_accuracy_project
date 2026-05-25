sim=require'sim'
simUI=require'simUI'

function handleUI(p)
    local s=sim.getObjectSel()
    if s and #s>0 and s[#s]==model then
        if not ui then
            local xml =[[<ui title="GPS" closeable="false" placement="relative" position="50,-50" layout="form">
                    <label text="x pos:" />
                    <label id="1" text="-" />
                    <label text="y pos:" />
                    <label id="2" text="-" />
                    <label text="z pos:" />
                    <label id="3" text="-" />
            </ui>]]
            ui=simUI.create(xml)
        end
        simUI.setLabelText(ui,1,string.format("%.3f",p[1]))
        simUI.setLabelText(ui,2,string.format("%.3f",p[2]))
        simUI.setLabelText(ui,3,string.format("%.3f",p[3]))
    else
        if ui then
            simUI.destroy(ui)
            ui=nil
        end
    end
end

function sysCall_init() 
    model=sim.getObject('..')
    ref=sim.getObject('../reference')

    xNoiseAmplitude=0
    yNoiseAmplitude=0
    zNoiseAmplitude=0
    xShift=0
    yShift=0
    zShift=0
end

function sysCall_sensing() 
    objectAbsolutePosition=sim.getObjectPosition(ref)
    
    -- Now add some noise to make it more realistic:
    objectAbsolutePosition[1]=objectAbsolutePosition[1]+2*(math.random()-0.5)*xNoiseAmplitude+xShift
    objectAbsolutePosition[2]=objectAbsolutePosition[2]+2*(math.random()-0.5)*yNoiseAmplitude+yShift
    objectAbsolutePosition[3]=objectAbsolutePosition[3]+2*(math.random()-0.5)*zNoiseAmplitude+zShift
    
    --Setting those float values in order for matlab to recognize
    sim.setFloatSignal('gpsX', objectAbsolutePosition[1])
    sim.setFloatSignal('gpsY', objectAbsolutePosition[2])
    sim.setFloatSignal('gpsZ', objectAbsolutePosition[3])
    handleUI(objectAbsolutePosition)
end 
