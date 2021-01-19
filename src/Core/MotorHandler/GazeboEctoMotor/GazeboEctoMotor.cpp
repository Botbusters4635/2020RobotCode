//
// Created by Neil Rodriguez on 21/08/20.
//

#include "GazeboEctoMotor.h"
#include <iostream>
#include <spdlog/sinks/stdout_color_sinks.h>

GazeboMotor::GazeboMotor(const std::string &modelName, const std::string &motorName) {
	
	log = spdlog::stdout_color_mt(modelName);
	
	this->modelName = modelName;
	this->motorName = motorName;
	modelTable = ntInstance.GetTable(modelName);
	motorTable = modelTable->GetSubTable(motorName);
	pidTable = motorTable->GetSubTable("pid");
	positionEntry = motorTable->GetEntry("position");
	velocityEntry = motorTable->GetEntry("velocity");
	setEntry = motorTable->GetEntry("setpoint");
	modeEntry = motorTable->GetEntry("mode");
	
}

double GazeboMotor::getPosition() {
	return positionEntry.GetDouble(0.0);
}

double GazeboMotor::getVelocity() {
	return velocityEntry.GetDouble(0.0);
}

double GazeboMotor::getPercent() {
	return setEntry.GetDouble(0.0);
}


void GazeboMotor::set(double value, MotorControlMode mode) {
	
	this->controlMode = mode;
	setEntry.SetDouble(value);
	
	switch (this->controlMode) {
		case MotorControlMode::Velocity:
			modeEntry.SetDouble(0.0);
			break;
		case MotorControlMode::Percent:
			 log->error("Percent can't be used in gazebo motor.");
			break;
		case MotorControlMode::Position:
			modeEntry.SetDouble(1.0);
			break;
	}
	
	
	
	
}
