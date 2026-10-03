-- Script do dockingSensor do robô: pede a leitura ao beacon e gira a seta
-- (/myRobot/dockingSensor/arrow) na direção da base, para dar o retorno visual
-- na cena.
--
-- CÓPIA DE REFERÊNCIA, extraída da cena com a Remote API. Editar aqui não muda
-- a cena. O comentário dele é a definição oficial da convenção do ângulo:
-- "direction in rad (0 is in front, i.e. x direction)" — o x DO SENSOR.


function sysCall_init()
    sim = require('sim')
    sensorHandle=sim.getObject('/dockingSensor')
    -- do some initialization here
    arrowHandle=sim.getObject('/arrow')
    oriarrow=sim.getObjectOrientation(arrowHandle,-1)
end

function sysCall_actuation()
    if sim.getSimulationTime()>0.5 then
    -- signal is strength in % and direction in rad (0 is in front, i.e. x direction) if the sensor is getting the signal
    -- if not signal and angle are nil
        local signal, angle = sim.callScriptFunction('getBeaconInfo',sim.getScript(sim.scripttype_simulation,"/chargingBase/beacon" ),sensorHandle)
        if angle~=nil then
            sim.setObjectOrientation(arrowHandle,sensorHandle,{oriarrow[1],oriarrow[2],angle})
        end
        if signal~=nil then
            --print("sensor is reading:",signal, angle)
        end
    end
end

