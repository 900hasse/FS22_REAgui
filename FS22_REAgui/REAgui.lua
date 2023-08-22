--
-- REA Script
-- author: 900Hasse
-- date: 23.11.2022
--
-- V1.0.0.0
--
-----------------------------------------
-- TO DO
---------------
-- 
-- 



-----------------------------------------
-- KNOWN ISSUES
---------------
-- 
-- 

print("---------------------------")
print("----- REA by 900Hasse -----")
print("---------------------------")
REAgui = {};

function REAgui.prerequisitesPresent(specializations)
    return true
end;

function REAgui:loadMap(name)
end

function REAgui:deleteMap()
end

function REAgui:draw(dt)

	-- If Client draw vehicle status on GUI
	local UseGUI = true;
	if UseGUI then
		if g_client and not g_gui:getIsGuiVisible() and not g_flightAndNoHUDKeysEnabled and g_currentMission.hud:getIsVisible() then
			-- Calculate time since last draw
			if REAgui.LastDrawTime == nil then
				REAgui.LastDrawTime = 0;
			end;
			-- Calculate time
			local TimeSinceLastDraw = g_currentMission.time - REAgui.LastDrawTime;
			-- Save time for next draw
			REAgui.LastDrawTime = g_currentMission.time;
			-- Check number of vehicles
			numVehicles = table.getn(g_currentMission.vehicles);
			-- If vehicles present run code
			if numVehicles ~= nil then
				-- Run code for vehicles
				if numVehicles >= 1 then
					-- Search for controlled vehicle
					for VehicleIndex=1, numVehicles do
						-- Save "vehicle" to local
						local vehicle = g_currentMission.vehicles[VehicleIndex];			
						-- Check if current vehicle exists
						if vehicle ~= nil then
							if vehicle.spec_motorized ~= nil then
								if vehicle.spec_wheels ~= nil then
									if vehicle:getIsControlled() then
										if g_currentMission.controlledVehicle == vehicle then
											REAgui:DrawStatus(vehicle,TimeSinceLastDraw);
											break
										end;
									end;
								end;
							end;
						end;
					end;
				end;
			end;
		end;
	end;
end;

function REAgui:update(dt)

	-----------------------------------------------------------------------------------
	-- Add REA functionality
	-----------------------------------------------------------------------------------
	if g_client and not g_gui:getIsGuiVisible() then
		-- Get number of vehicles
		local numVehicles = table.getn(g_currentMission.vehicles);
		-- If vehicles present run code
		if numVehicles ~= nil then
			-- Run code for vehicles
			if numVehicles >= 1 then
				for VehicleIndex=1, numVehicles do
					-- Save "vehicle" to local
					local vehicle = g_currentMission.vehicles[VehicleIndex];			
					-- Check if current vehicle exists
					if vehicle ~= nil then
						if vehicle.spec_motorized ~= nil then
							if vehicle.spec_wheels ~= nil then	
								if vehicle.spec_wheels.wheels ~= nil then	
									if vehicle:getIsControlled() then
										if g_currentMission.controlledVehicle == vehicle then
											local numWheels = table.getn(vehicle.spec_wheels.wheels);
											for WheelIndex=1,numWheels do
												-- Save wheel to local
												local wheel = vehicle.spec_wheels.wheels[WheelIndex];
												if g_server == nil then
													-- Update speed calculated from xDrive
													REAgui:WheelSpeedFromXdrive(wheel,dt);
													-- Update sideway speed and direction of active wheel
													REAgui:UpdateWheelDirectionAndSpeed(wheel,dt);
													-- Update wheel distance based on xDrive
													REAgui:WheelDistanceFromXdrive(wheel,dt)
												end;
												-- Calculate slip
												REAgui:CalcSlip(vehicle,dt);
											end;
										end;
									end;
								end;
							end;
						end;
					end;
				end;
			end;
		end;
	end;
end;



-----------------------------------------------------------------------------------	
-- Draw status of vehicle
-----------------------------------------------------------------------------------
function REAgui:DrawStatus(vehicle,dt)

	--------------------------------------------------------------------
	-- Init global variables
	--------------------------------------------------------------------
	if vehicle.timer == nil or vehicle.GUISlip == nil then
		vehicle.timer = 0;
		vehicle.GUIMotorLoad = 0;
		vehicle.GUISlip = 0;
		vehicle.GUIExternalPowerNeed = 0;
	end;
	
	--------------------------------------------------------------------
	-- Get engine Load
	--------------------------------------------------------------------
	local Slip = 0;
	if vehicle.spec_wheels.SlipSmoothed ~= nil then
		Slip = REAgui:RoundValue(vehicle.spec_wheels.SlipSmoothed);
		-- Slip is 0-100%
		if Slip > 100 then
			Slip = 100;
		elseif Slip < 0 then
			Slip = 0;
		end
	end;

	--------------------------------------------------------------------
	-- Get engine Load
	--------------------------------------------------------------------
	-- Calculate motor load
	local MotorLoad = REAgui:RoundValue(vehicle.spec_motorized.smoothedLoadPercentage * 100);
	-- Motor load is 0-100%
	if MotorLoad > 100 then
		MotorLoad = 100;
	-- elseif MotorLoad < 0 then
	-- 	MotorLoad = 0;
	end


	--------------------------------------------------------------------
	-- Get external powerneed
	--------------------------------------------------------------------
	-- Calculate power need
	local spec = vehicle.spec_motorized;
	local ExternalPower = REAgui:RoundValue(spec.motor.motorExternalTorque * spec.motor:getEqualizedMotorRpm() * math.pi / 30);

	--------------------------------------------------------------------
	-- Uppdate every 50ms
	if vehicle.timer > 50 then
		--------------------------------------------------------------------
		-- slip
		if Slip ~= nil then
			vehicle.GUISlip = Slip;
		end;
		-- Motor load
		if MotorLoad ~= nil then
			vehicle.GUIMotorLoad = MotorLoad;
		end;
		-- External power need
		if ExternalPower ~= nil then
			vehicle.GUIExternalPowerNeed = ExternalPower;
		end;
		-- Reset timer
		vehicle.timer = 0;
	end;
	-- Add time
	vehicle.timer = vehicle.timer + dt;

	--------------------------------------------------------------------
	-- Prepare overlays and text
	--------------------------------------------------------------------
	-- Create overlays
	local OverlaySlip = 1;
	local OverlayLoad = 2;
	-- Transparancy
	local Transparancy = 0.6;
	-- Create overlay for slip
	if REAgui.overlay[OverlaySlip] == nil then
		table.insert(REAgui.overlay,createImageOverlay(REAgui.FilePath .. "media/SLIP_ICON.dds"));
		setOverlayColor(REAgui.overlay[OverlaySlip], 0, 1, 0, Transparancy);
	end;
	-- Create overlay for load
	if REAgui.overlay[OverlayLoad] == nil then
		table.insert(REAgui.overlay,createImageOverlay(REAgui.FilePath .. "media/LOAD_ICON.dds"));
		setOverlayColor(REAgui.overlay[OverlayLoad], 0, 1, 0, Transparancy);
	end;

	-- Text settings
	local FontSize = 0.017;
	local TextPadding = 0.001;
	UiScale = 1;
	if g_gameSettings.uiScale ~= nil then
		UiScale = g_gameSettings.uiScale;
	end
	local Width = ((FontSize + TextPadding) * 0.8 ) * UiScale;
	local Height = (((FontSize + TextPadding) * 0.8 ) * 2 ) * UiScale;
	local FontsizeWithScale = FontSize * UiScale;

	-- Get speed gauge center
	local SpeedGaugeCenterX = g_currentMission.inGameMenu.hud.speedMeter.gaugeCenterX
	local SpeedGaugeCenterY = g_currentMission.inGameMenu.hud.speedMeter.gaugeCenterY
	local posY = SpeedGaugeCenterY * 0.05;
	local LoadPosX = SpeedGaugeCenterX * (1 - (0.10 * UiScale));
	local SlipPosX = SpeedGaugeCenterX * (1 - (0.15 * UiScale));
	local PowerPosX = SpeedGaugeCenterX * (1 - (0.20 * UiScale));

	--------------------------------------------------------------------
	-- Write GUI
	-- Render overlay
	renderOverlay(REAgui.overlay[OverlaySlip], SlipPosX, posY, Width, Height);
	renderOverlay(REAgui.overlay[OverlayLoad], LoadPosX, posY, Width, Height);
	-- Render text
	setTextColor(0,1,0,1)
	setTextAlignment(RenderText.ALIGN_LEFT)
	setTextVerticalAlignment(RenderText.VERTICAL_ALIGN_BOTTOM)
	setTextBold(false)
	renderText(SlipPosX + Width + (TextPadding * 2), posY + (Height / 2) - (FontsizeWithScale / 2), FontsizeWithScale, vehicle.GUISlip .. " %")
	renderText(LoadPosX + Width + (TextPadding * 2), posY + (Height / 2) - (FontsizeWithScale / 2), FontsizeWithScale, vehicle.GUIMotorLoad .. " %")
	renderText(PowerPosX + Width + (TextPadding * 2), posY + (Height / 2) - (FontsizeWithScale / 2), FontsizeWithScale, vehicle.GUIExternalPowerNeed .. " hp")
	-- Thanks to Mogli fixing this
	setTextAlignment( RenderText.ALIGN_LEFT ) 
	setTextVerticalAlignment( RenderText.VERTICAL_ALIGN_BASELINE )
	setTextColor(1, 1, 1, 1) 

end

--------------------------------------------------------------------
-- Function to calculate slip
--------------------------------------------------------------------
function REAgui:CalcSlip(vehicle,dt)
	-- How many wheels do the vehicle have
	local numWheels = table.getn(vehicle.spec_wheels.wheels);
	-- Get speed
	local VehicleSpeed = vehicle:getLastSpeed();
	-- Loop to get average speed of all wheels
	local TotalWheelSpeed = 0;
	for Wheel=1,numWheels do
		-- Save active wheel to local wheel
		local Actwheel = vehicle.spec_wheels.wheels[Wheel];
		-- Get speed of wheel
		-- If speed was not calculated by server calculate speed based on xDrive
		if Actwheel.SpeedBasedOnXdrive ~= nil then
			TotalWheelSpeed = Actwheel.SpeedBasedOnXdrive + TotalWheelSpeed;			
		end;
	end;
	-- Smoothe average wheelspeed
	if vehicle.spec_wheels.AverageSpeedSmoothed == nil then
		vehicle.spec_wheels.AverageSpeedSmoothed = 0;
	end;
	vehicle.spec_wheels.AverageSpeedSmoothed = REAgui:SmootheValue(vehicle.spec_wheels.AverageSpeedSmoothed,TotalWheelSpeed / numWheels);
	-- Calculate slip
	if vehicle.spec_wheels.AverageSpeedSmoothed > 0.2 then
		-- Calculate differance
		local SpeedDiff = math.abs(VehicleSpeed - vehicle.spec_wheels.AverageSpeedSmoothed);
		if SpeedDiff > 0.2 and VehicleSpeed < vehicle.spec_wheels.AverageSpeedSmoothed then
			-- Calculate slip
			local Slip = (SpeedDiff / vehicle.spec_wheels.AverageSpeedSmoothed) * 100;
			vehicle.spec_wheels.SlipSmoothed = REAgui:RoundValue(REAgui:SmootheValue(vehicle.spec_wheels.SlipSmoothed,Slip));
		else
			vehicle.spec_wheels.SlipSmoothed = 0;
		end;
	else
		vehicle.spec_wheels.SlipSmoothed = 0;
	end;
end;


-----------------------------------------------------------------------------------	
-- Function to round value, delete decimals
-----------------------------------------------------------------------------------
function REAgui:RoundValue(x)
	return x>=0 and math.floor(x+0.5) or math.ceil(x-0.5)
end


-----------------------------------------------------------------------------------	
-- Function to round value with two decimals
-----------------------------------------------------------------------------------
function REAgui:RoundValueTwoDecimals(x)
	x = x*100;
	x = x>=0 and math.floor(x+0.5) or math.ceil(x-0.5);
	x = x/100;
	return x;
end


-----------------------------------------------------------------------------------	
-- Function to get wheelslip
-----------------------------------------------------------------------------------
function REAgui:GetWheelSlipFactor(WheelSpeed,VehicleSpeed)
	local WheelSlip = 0;
	if WheelSpeed > 1 then
		-- Calculate differance
		local SpeedDiff = math.abs(VehicleSpeed - WheelSpeed);
		if SpeedDiff > 1 and VehicleSpeed < WheelSpeed then
			-- Calculate slip
			WheelSlip = REAgui:RoundValueTwoDecimals(SpeedDiff / WheelSpeed);
		end;
	end;
	return MathUtil.clamp(WheelSlip, 0, 1);
end


-----------------------------------------------------------------------------------	
-- Function to smoothe value
-----------------------------------------------------------------------------------
function REAgui:SmootheValue(SmoothedValue,RealValue)
	-- If no smoothevalue use the real value
	if SmoothedValue == nil then
		ActValue = RealValue;
	else
		ActValue = SmoothedValue;
	end;
	-- Return the smoothed value
	return (ActValue*0.9)+(RealValue*0.1);
end


-----------------------------------------------------------------------------------	
-- Function to calculate speed based on xDrive(wheel position)
-----------------------------------------------------------------------------------
function REAgui:WheelSpeedFromXdrive(wheel,dt)
	-- initialize last xDrive
	if wheel.xDriveLast == nil then
		wheel.xDriveLast = 0;
		wheel.xDriveLastKMH = 0;
		wheel.xDriveDirection = 0;
		wheel.xDriveLastDirection = 0;
		wheel.xDriveSignedSpeedSmoothed = 0;
	end;
	-- Get differance from last call
	local RadDiff = wheel.netInfo.xDrive - wheel.xDriveLast;
	-- Save last xDrive
	wheel.xDriveLast = wheel.netInfo.xDrive;
	-- If wheel starts a new turn assume that the speed is constant and return last calulated speed
	if math.abs(RadDiff) > 3.14 then
		-- Return speed in KMH
		wheel.SpeedBasedOnXdrive = wheel.xDriveLastKMH;
		wheel.DirectionBasedOnXdrive = wheel.xDriveLastDirection;
	-- If not a new turn calculate a neww speed
	else
		-- Calculate speed
		local DistanceTraveled = REAgui:RoundValueTwoDecimals(RadDiff * wheel.radiusOriginal);
		local MeterPerSecond = DistanceTraveled/(dt/1000);
		-- Convert to KMH
		local WheelSpeedSigned = MeterPerSecond*3.6;
		-- Smoothe value
		wheel.xDriveSignedSpeedSmoothed =  (wheel.xDriveSignedSpeedSmoothed*0.7)+(WheelSpeedSigned*0.3);
		-- Save direction of wheel rotation
		local WheelSpeedSignedSmoothed = wheel.xDriveSignedSpeedSmoothed;
		local WheelSpeedSmoothed = math.abs(WheelSpeedSignedSmoothed);
		local MinSpeedForUpdate = 0.25;
		if WheelSpeedSmoothed >= MinSpeedForUpdate then
			if WheelSpeedSignedSmoothed > 0 then
				wheel.xDriveDirection = 1;
			elseif WheelSpeedSignedSmoothed < 0 then
				wheel.xDriveDirection = -1;
			end;
		elseif WheelSpeedSmoothed < MinSpeedForUpdate then
			wheel.xDriveDirection = 0;
			wheel.xDriveSignedSpeedSmoothed = 0;
		end;
		-- Save speed if wheel starts a new turn
		wheel.xDriveLastKMH = WheelSpeedSmoothed;
		wheel.xDriveLastDirection = wheel.xDriveDirection;
		-- Return speed in KMH
		wheel.SpeedBasedOnXdrive = WheelSpeedSmoothed;
		wheel.DirectionBasedOnXdrive = wheel.xDriveDirection;
	end;
end


-----------------------------------------------------------------------------------	
-- Function to calculate expected and actual moved distance of wheel 
-----------------------------------------------------------------------------------
function REAgui:WheelDistanceFromXdrive(wheel,dt)
	-- initialize last xDrive
	if wheel.DistancexDriveLast == nil then
		wheel.DistanceLastPosition = {0,0,0};
		wheel.DistancexDriveLast = 0;
	end;
	-- Get position of wheel
	local x,y,z = getWorldTranslation(wheel.repr);
	-- Calculate differance in position from last call
	local dx, dy, dz = worldDirectionToLocal(wheel.repr, x-wheel.DistanceLastPosition[1], y-wheel.DistanceLastPosition[2], z-wheel.DistanceLastPosition[3]);
	-- Save position for next call
	wheel.DistanceLastPosition[1], wheel.DistanceLastPosition[2], wheel.DistanceLastPosition[3] = x, y, z;

	-- Get differance from last call
	local RadDiff = math.abs(wheel.DistancexDriveLast - wheel.netInfo.xDrive);
	-- Save last xDrive
	wheel.DistancexDriveLast = wheel.netInfo.xDrive;
	-- If wheel starts a new turn assume no change and return zero change
	if RadDiff > 3.14 then
		wheel.ExpectedDistanceTraveled = 0;
		wheel.ActualDistanceTraveled = 0;
	-- If not a new turn calculate distance traveled
	else
		-- Calculate expected moved distance
		wheel.ExpectedDistanceTraveled = RadDiff * wheel.radiusOriginal;
		wheel.ActualDistanceTraveled = MathUtil.vector3Length(dx, dy, dz)
	end;
end


-----------------------------------------------------------------------------------	
-- Calculate sideway speed of wheel
-----------------------------------------------------------------------------------
function REAgui:UpdateWheelDirectionAndSpeed(wheel,dt)
	-- Create node to calculate speed and direction
	if wheel.REASpeedNode == nil then
		local REASpeedNode = createTransformGroup("REASpeedNode")
		link(wheel.node, REASpeedNode);
		setTranslation(REASpeedNode, getTranslation(wheel.driveNode));
		setRotation(REASpeedNode,0,0,0);
		setScale(REASpeedNode, 1, 1, 1)
		wheel.REASpeedNode = REASpeedNode;
	end;
	-- Update steeting angle of wheel
	if wheel.steeringAngle ~= 0 then 
		setRotation(wheel.REASpeedNode,0,wheel.steeringAngle,0);
	end;

	-- Calculate speed based on the position change
	local x,y,z = getWorldTranslation(wheel.REASpeedNode);
	if wheel.REASpeedLastPosition == nil then
		wheel.REASpeedLastPosition = {x,y,z};
	end;
	local SpeedDiffX, SpeedDiffY, SpeedDiffZ = worldDirectionToLocal(wheel.REASpeedNode, x-wheel.REASpeedLastPosition[1], y-wheel.REASpeedLastPosition[2], z-wheel.REASpeedLastPosition[3]);
	wheel.REASpeedLastPosition[1], wheel.REASpeedLastPosition[2], wheel.REASpeedLastPosition[3] = x, y, z;

	-- Rolling direction, calculate speed of wheel in direction
	wheel.RollingDirectionSpeed, wheel.RollingDirectionSpeedSigned, wheel.RollingMovingDirection = REAgui:CalcSpeedAndDirection(wheel.RollingDirectionSpeedSigned,SpeedDiffZ,dt)
	-- SideWay direction, calculate speed of wheel in direction
	wheel.SideWaySpeed, wheel.SideWaySpeedSigned, wheel.SideWayMovingDirection = REAgui:CalcSpeedAndDirection(wheel.SideWaySpeedSigned,SpeedDiffX,dt)

end;


-----------------------------------------------------------------------------------	
-- Function to calculate speed and direcrtion based on movement
-----------------------------------------------------------------------------------
function REAgui:CalcSpeedAndDirection(LastSpeedSigned,MovedDistance,dt)
	-- Calculate speed of wheel
	local SpeedSigned = (MovedDistance / dt)*3600;
	local SpeedSignedSmoothe = REAgui:SmootheValue(LastSpeedSigned,SpeedSigned)
	local Speed = SpeedSignedSmoothe;
	-- Remove sign
	if Speed < 0 then
		Speed = Speed*(-1); 
	end;
	-- Moving direction
	local MovingDirection = 0;
	if SpeedSigned > 0.01 then
		MovingDirection = 1;
	elseif SpeedSigned < -0.01 then
		MovingDirection = -1;
	end;
	-- Return result
	return Speed, SpeedSignedSmoothe, MovingDirection;
end


---
function Motorized:onReadUpdateStream(streamId, timestamp, connection)
    local spec = self.spec_motorized
    if connection.isServer then
        if streamReadBool(streamId) then
            local rpm = streamReadUIntN(streamId, 11) / 2047
            local rpmRange = spec.motor:getMaxRpm()- spec.motor:getMinRpm()
            spec.motor:setEqualizedMotorRpm( (rpm * rpmRange) + spec.motor:getMinRpm() )

            local loadPercentage = streamReadUIntN(streamId, 7)
            spec.motor.rawLoadPercentage = loadPercentage / 127

            local ExternalTourqe = streamReadUIntN(streamId, 7)
            spec.motor.motorExternalTorque = ExternalTourqe / 127

            spec.brakeCompressor.doFill = streamReadBool(streamId)

            local clutchState = streamReadUIntN(streamId, 5)
            spec.motor:onManualClutchChanged(clutchState / 31)
        end

        if streamReadBool(streamId) then
            spec.motor:readGearDataFromStream(streamId)
        end
    else
        if streamReadBool(streamId) then
            if streamReadBool(streamId) then
                spec.clutchState = streamReadUIntN(streamId, 7) / 127

                spec.motor:onManualClutchChanged(spec.clutchState)

                if spec.clutchState > 0 then
                    self:raiseActive()
                end
            end
        end
    end
end


---
function Motorized:onWriteUpdateStream(streamId, connection, dirtyMask)
    local spec = self.spec_motorized
    if not connection.isServer then
        if streamWriteBool(streamId, self.spec_motorized.isMotorStarted) then
            local rpmRange = spec.motor:getMaxRpm() - spec.motor:getMinRpm()
            local rpm = MathUtil.clamp((spec.motor:getEqualizedMotorRpm() - spec.motor:getMinRpm()) / rpmRange, 0, 1)

            rpm = math.floor(rpm * 2047)
            streamWriteUIntN(streamId, rpm, 11)
            streamWriteUIntN(streamId, 127 * spec.actualLoadPercentage, 7)
            streamWriteUIntN(streamId, 127 * spec.motor.motorExternalTorque, 7)
            streamWriteBool(streamId, spec.brakeCompressor.doFill)
            streamWriteUIntN(streamId, 31 * spec.motor:getClutchPedal(), 5)
        end

        if streamWriteBool(streamId, bitAND(dirtyMask, spec.dirtyFlag) ~= 0) then
            spec.motor:writeGearDataToStream(streamId)
        end
    else
        if streamWriteBool(streamId, bitAND(dirtyMask, spec.inputDirtyFlag) ~= 0) then
            if streamWriteBool(streamId, spec.clutchState ~= spec.clutchStateSent) then
                streamWriteUIntN(streamId, 127 * spec.clutchState, 7)
                spec.clutchStateSent = spec.clutchState
            end
        end
    end
end


if REAgui.ModActivated == nil then

	addModEventListener(REAgui);

	--WORK
	REAgui.ModActivated = true;
	REAgui.FilePath = g_currentModDirectory;
	REAgui.overlay = {};

	print("mod activated")


end;
