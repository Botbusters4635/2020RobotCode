//
// Created by Neil Rodriguez on 21/08/20.
//

#ifndef BOTBUSTERS_REBIRTH_GAZEBOECTOMOTOR_H
#define BOTBUSTERS_REBIRTH_GAZEBOECTOMOTOR_H

#include <networktables/NetworkTableInstance.h>
#include "Core/MotorHandler/EctoMotor/DataTypes/EctoMotorMode.h"
#include "spdlog/spdlog.h"


class GazeboMotor {

public:
	GazeboMotor(const std::string &modelName, const std::string &motorName);
	
	double getPosition();
	
	double getVelocity();
	
	double getPercent();
	
	void set(double value, MotorControlMode mode);
	
private:
	
	
	std::string modelName;
	
	std::string motorName;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	
	std::shared_ptr<NetworkTable> modelTable;
	
	std::shared_ptr<NetworkTable> motorTable;
	
	std::shared_ptr<NetworkTable> pidTable;
	
	nt::NetworkTableEntry positionEntry;
	
	nt::NetworkTableEntry velocityEntry;
	
	nt::NetworkTableEntry setEntry;
	
	nt::NetworkTableEntry modeEntry;
	
	

protected:
	MotorControlMode controlMode = MotorControlMode::Percent;
	
	std::shared_ptr<spdlog::logger> log;
	
};


#endif //BOTBUSTERS_REBIRTH_GAZEBOECTOMOTOR_H
