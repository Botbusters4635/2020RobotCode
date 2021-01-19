//
// Created by alberto on 16/09/19.
//

#include "BallIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

BallIntake::BallIntake(const std::shared_ptr<EctoMotor> &leftMotor, const std::shared_ptr<EctoMotor> &rightMotor)
		: System("BallIntake") {
	leftIntakeMotor = leftMotor;
	rightIntakeMotor = rightMotor;
	
	leftMotor->setOpenLoopRampRate(0.08);
	rightIntakeMotor->setOpenLoopRampRate(0.08);
}

void BallIntake::initRobot() {
	leftPiston = pcm.getPiston("left_intake");
	rightPiston = pcm.getPiston("right_intake");
}

void BallIntake::updateRobot() {
	;
}

void BallIntake::setLeftIntakeState(bool state, double setpoint) {
	PCMManager::setPistonState(leftPiston, state);
	leftIntakeMotor->set(setpoint);
}

void BallIntake::setRightIntakeState(bool state, double setpoint) {
	PCMManager::setPistonState(rightPiston, state);
	rightIntakeMotor->set(-setpoint);
}