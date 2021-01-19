//
// Created by abiel on 2/22/20.
//

#include "TeleopPathHelper.h"

TeleopPathHelper::TeleopPathHelper(const shared_ptr<EctoSwerve> &swerve) : System("TeleopPathHelper", false) {
	this->swerve = swerve;
	
	input.registerButton(&logButton, "X");
}

void TeleopPathHelper::updateRobot() {
	if (logButton.get() and logButton.get() != buttonLastPressed) {
		const auto currentPose = swerve->getPose();
		log->info("X:{}m, Y:{}m Theta:{}rad", currentPose.getX(), currentPose.getY(), currentPose.getTheta());
	}
	
	buttonLastPressed = logButton.get();
}