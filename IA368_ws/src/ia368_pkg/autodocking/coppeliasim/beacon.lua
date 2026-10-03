-- Script da base de carga: responde com a informação do beacon IR quando
-- chamado, e liga/desliga o sinal de carga do robô.
--
-- CÓPIA DE REFERÊNCIA, extraída da cena com a Remote API. Editar aqui não muda
-- a cena; serve para ler e entender. É o script que define TODO o protocolo que
-- o charging_base_node e o autodocking usam, em particular:
--
--   * relativeAngle = atan2(base - sensor) - orientação DO dockingSensor,
--     normalizado em -pi..pi. O "0 = à frente" é a frente do SENSOR; nesta cena
--     ela coincide com a do robô (a frente do robô é o +y dele, e o sensor está
--     no nariz com o +x apontando para lá), então angle_target = 0;
--   * signalStrength = 1 - distância / volume_range  (volume_range = 2 m);
--   * o ângulo e a força só são escritos se o objeto detectado pelo feixe for
--     EXATAMENTE o dockingSensor — perto da base o para-choque passa à frente
--     e o beacon se cala (ver o estado FINAL do autodocking.py);
--   * a carga (<h>Charging = 1) exige distância <= 0,1 m e é verificada só 1x
--     por segundo;
--   * o feixe é um corredor ESTREITO: medido na cena, só há detecção numa única
--     direção a partir da base (bearing 90°), até ~1,2-1,5 m. Fora dele o
--     beacon não responde nada.


-- Charging Base Script: provides beacon info when called and if the caller sensor is in range

function sysCall_init()
    baseHandle = sim.getObject("/chargingBase")
    beaconHandle=sim.getObject("/ir_beam")
    beaconOri=sim.getObjectOrientation(beaconHandle)
    
    -- beacon inits
    maxSignalDistance = sim.getFloatProperty(beaconHandle,"volume_range")  -- max beacon range in meters
    minChargeingDist = 0.1 -- m , minimum distance that the robot needs to be to the charger (in range) to start charging
    
    -- charging inits
    charging=false
    sensorHandle = beaconHandle
    initTime=sim.getSimulationTime()

end

-- This function will be called by the Roomba
function getBeaconInfo(dockingSensorHandle)
    local basePos = sim.getObjectPosition(baseHandle, -1)
    local roombaPos = sim.getObjectPosition(dockingSensorHandle, -1)
    local roombaOri = sim.getObjectOrientation(dockingSensorHandle, -1)  -- assuming rotation around Z

    local dx = basePos[1] - roombaPos[1]
    local dy = basePos[2] - roombaPos[2]

    local angleToBase = math.atan2(dy, dx)
    local relativeAngle = angleToBase - roombaOri[3]

    -- normalize relative angle to [-pi, pi]
    while relativeAngle > math.pi do relativeAngle = relativeAngle - 2*math.pi end
    while relativeAngle < -math.pi do relativeAngle = relativeAngle + 2*math.pi end

    --testing if the object sensor is in range
    state,distance, detectPoint,detectedObjectHandle=sim.readProximitySensor(beaconHandle)
    signalStrength= 1 - distance / maxSignalDistance
    if detectedObjectHandle== dockingSensorHandle then
        return signalStrength, relativeAngle
    else
        return nil,nil
    end
end



function sysCall_actuation()
---TEST TO REMOVE
--sim.setInt32Signal("Beacon",sim.getObject("/myRobot"))
----
    --check if beacon data is asked from broadcasted beacon signal
    roombaHandle=sim.getInt32Signal("Beacon")
    if roombaHandle ~= nil then
        -- send packed float array {signalStrength, relativeAngle} in string signal roombaHandle.."BeaconInfo" (ex 1234BeaconInfo)
        -- on the other end of the signal, the string needs to be sim.unpackFloatTable to get the float table. 
        -- CAUTION getBeaconInfo gets the handle of the DockingSensor object and not the myRobot?s one
        local robot = sim.getObject("/"..sim.getObjectAlias(roombaHandle).."/dockingSensor")
        if robot~= nil then
            local sigStrength,relAng=getBeaconInfo(robot)
            if sigStrength~= nil then
                --sim.setStringSignal(roombaHandle.."BeaconInfo",sim.packFloatTable({sigStrength,relAng}))
                sim.setFloatSignal(roombaHandle.."StrengthSignal", sigStrength)
                sim.setFloatSignal(roombaHandle.."RelativeAngle", relAng)
            end
        end
---TEST TO REMOVE
--[[        local data=sim.getStringSignal(roombaHandle.."BeaconInfo")
        if data~=nil then
            sim.clearStringSignal(roombaHandle.."BeaconInfo")
            print("beacon: strength and relAngle",sim.unpackFloatTable(data))
        end
--]]
--------------
    end
    

    
    if sim.getSimulationTime()-initTime > 1 then --checking at every 1 sec
        initTime=sim.getSimulationTime()
        res,dist,dp,detectedHandle=sim.readProximitySensor(sensorHandle)
        -- detected handle?s father?s handels is supposed to be the myRobot?s handle 
        detectedHandle=sim.getObjectParent(detectedHandle)
        
        print("beacon: ",res, sim.getObjectAlias(detectedHandle),checkIfRobot(detectedHandle) )
        --we are detecting a robot, so lets lets put it in charging mode if dist<= minChargeingDist  
        if res==1 and detectedHandle~=nil and dist <= minChargeingDist then
            if checkIfRobot(detectedHandle) then 
                print("beacon: charging "..detectedHandle.."Charging")
                sim.setInt32Signal(detectedHandle.."Charging",1)
                lastDetectedHandle=detectedHandle
                charging=true
            end
        end   
        if charging==true and (checkIfRobot(detectedHandle)==false or dist> minChargeingDist) then 
            -- here we were charging but the robot is not in the base anymore
            charging=false
            sim.setInt32Signal(lastDetectedHandle.."Charging",0)
        end

    end
end

local function starts_with(str, start)
   return str:sub(1, #start) == start
end

function checkIfRobot(detectedHandle)
    -- checking if the detectedHandle belongs to a myRobot
    if detectedHandle==nil then return false end
    name=sim.getObjectAlias(detectedHandle)
    return starts_with( name, "myRobot" )  
end




