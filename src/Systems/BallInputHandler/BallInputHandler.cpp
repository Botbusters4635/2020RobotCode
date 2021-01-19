//
// Created by abiel on 2/10/20.
//

#include "BallInputHandler.h"
#include <frc/DriverStation.h>

BallInputHandler::BallInputHandler(const BallInputHandlerConfig &config) : System("BallInputHandler", false) {
	this->config = config;
	input.registerButton(&stopBallsOnSwitch, "B", 2);
	
	input.registerButton(&leftIntakeEnable, "leftPush", 2);
	input.registerButton(&rightIntakeEnable, "rightPush", 2);
	
	input.registerButton(&closeShoot, "A");
	input.registerButton(&lessShoot, "Y");
	input.registerButton(&longShoot, "B");
	input.registerButton(&stopFlywheel, "X");
	
	input.registerAxis(&ballFeeder, "leftX", 2);
	input.registerAxis(&intakeSpeedControl, "rightX", 2);
	
	table = ntInstance.GetTable("BallInputHandler");
	elevatorTable = ntInstance.GetTable("ElevatorInputHandler");
	swerveTable = ntInstance.GetTable("EctoSwerve");
}

void BallInputHandler::initRobot() {
	;
}

void BallInputHandler::updateRobot() {
	const double robotYaw = swerveTable->GetEntry("EctoSwerve/Odometry/Pose/Heading").GetDouble(0);
	
	if (!stopBallsOnSwitch.get()) {
		//Stop on default
		config.feeder->stopBallAtSwitch(true);
		
		config.feeder->feedBalls(-ballFeeder.get() * config.maxVelOnSwitchIntake);
	} else {
		//Stop on default
		config.feeder->stopBallAtSwitch(false);
		
		config.feeder->feedBalls(-ballFeeder.get());
	}
	
	if (closeShoot.get()) {
		config.shooter->setHood(HoodPosition::CloseRange);
		config.shooter->spinup(312);
	} else if (longShoot.get()) {
		config.shooter->setHood(HoodPosition::LongRange);
		config.shooter->spinup(620);
	} else if (lessShoot.get()) {
		config.shooter->setHood(HoodPosition::CloseRange);
		config.shooter->spinup(305);
	}
	
	if (stopFlywheel.get()) {
		config.shooter->spinup(0);
	}
	
	
	bool elevatorToggleState = elevatorTable->GetEntry("ElevatorToggle").GetBoolean(false);
	
	if (!elevatorToggleState) {
		config.intake->setLeftIntakeState(leftIntakeEnable.get(),
		                                  intakeSpeedControl.get());
		
		config.intake->setRightIntakeState(rightIntakeEnable.get(),
		                                   intakeSpeedControl.get());
		
		//input.setControllerRumble(std::abs(leftIntakeEnable.get() ? intakeSpeedControl.get() * 0.5 : 0), std::abs(rightIntakeEnable.get() ? intakeSpeedControl.get() * 0.5 : 0), 1);
		input.setControllerRumble(std::abs(leftIntakeEnable.get() ? intakeSpeedControl.get() * 0.5 : 0),
		                          std::abs(rightIntakeEnable.get() ? intakeSpeedControl.get() * 0.5 : 0), 2);
	} else {
		input.setControllerRumble(std::abs(intakeSpeedControl.get()), std::abs(intakeSpeedControl.get()), 2);
		config.intake->setLeftIntakeState(false,
		                                  0);
		
		config.intake->setRightIntakeState(false,
		                                   0);
	}
	
	const double matchTime = frc::DriverStation::GetInstance().GetMatchTime();
	
	if (frc::DriverStation::GetInstance().IsFMSAttached() and matchTime < 30) {
		input.setControllerRumble(0.1, 0.1, 1);
	}
}