//
// Created by abiel on 2/19/20.
//

#include "WpiTrajectoryVisionFollower.h"

WPITrajectoryVisionFollower::WPITrajectoryVisionFollower(const shared_ptr<EctoSwerve> &swerveIn,
                                                         WPITrajectoryFollowerConfig config,
                                                         const frc::Trajectory &trajectory, double defaultAngle) {
	trajectoryFollower = std::make_unique<WPITrajectoryFollower>(swerveIn, config, trajectory, defaultAngle);
	this->swerve = swerveIn;
}

void WPITrajectoryVisionFollower::Initialize() {
	trajectoryFollower->Initialize();
}

void WPITrajectoryVisionFollower::Execute() {
	const auto latestMessage = manager.getLatestData();
	
	if (latestMessage.isDetected) {
		const double targetAngle = EctoMath::wrapAngle(swerve->getYaw() - latestMessage.robotPose.getHeading().getRadians());
		trajectoryFollower->setTargetYaw(targetAngle);
	}
}

void WPITrajectoryVisionFollower::End(bool interrupted) {
	trajectoryFollower->End(interrupted);
}

bool WPITrajectoryVisionFollower::IsFinished() {
	return trajectoryFollower->IsFinished();
}