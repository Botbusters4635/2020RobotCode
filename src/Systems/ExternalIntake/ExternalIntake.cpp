//
// Created by abiel on 1/6/20.
//

#include "ExternalIntake.h"

ExternalIntake::ExternalIntake(const ExternalIntakeConfig &config, std::string externalIntakeName) : System(
		externalIntakeName) {
	this->config = config;
	this->externalIntakeName = externalIntakeName;
	piston = pcm.getPiston(config.pistonName);
	
	intakeMotor = config.intakeMotor;
}

void ExternalIntake::liftIntake(bool state) {
	pcm.setPistonState(piston, config.invertPiston == !state);
}

void ExternalIntake::setMotor(double value) {
	intakeMotor->set(value);
}

void ExternalIntake::initRobot() {
	intakeMotor->setMotorCurrentLimit(config.motorCurrentLimit);
	intakeMotor->enableCurrentLimit(true);
	intakeMotor->setOpenLoopRampRate(config.motorOpenLoopRampRate);
	intakeMotor->setControlMode(MotorControlMode::Percent);
	
	table = ntInstance.GetTable(externalIntakeName);
}

bool ExternalIntake::isIntakeDown() const {
	return piston.currentState;
}

void ExternalIntake::updateNetworkTables() {
	table->GetEntry("CurrentSetpoint").SetDouble(intakeMotor->getLastSetpoint().second);
	table->GetEntry("IsIntakeDown").SetBoolean(isIntakeDown());
}

void ExternalIntake::updateRobot() {
	updateNetworkTables();
}