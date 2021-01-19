//
// Created by abiel on 2/17/20.
//

#include "LowerIntake.h"

LowerIntake::LowerIntake(const std::shared_ptr<BallIntake> &ballIntake, const LowerIntakeSetpoint &setpoint) {
	this->ballIntake = ballIntake;
	this->setpoint = setpoint;
	
	this->SetName("LowerIntake");
}

void LowerIntake::Initialize() {
	ballIntake->setLeftIntakeState(setpoint.lowerLeft, setpoint.leftSetpoint);
	ballIntake->setRightIntakeState(setpoint.lowerRight, setpoint.rightSetpoint);
}

void LowerIntake::Execute() {
	;
}

void LowerIntake::End(bool interrupted) {
	;
}

bool LowerIntake::IsFinished() {
	return true;
}