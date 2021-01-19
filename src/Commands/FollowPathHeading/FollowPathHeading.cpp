//
// Created by abiel on 1/4/20.
//

#include "FollowPathHeading.h"

FollowPathHeading::FollowPathHeading(std::shared_ptr<EctoSwerve> swerve, const Path &path, double targetHeading,
                                     const FollowPathHeadingConfig &config) {
	this->swerve = swerve;
	
	pathPlanner = std::make_shared<SimplePathPlanner>(path, config.lookaheadDistance, config.finishThreshold);
	pathFollower = std::make_unique<PIDHolonomicPathFollower>(config.pathFollowerConfig, pathPlanner);
	
	this->profileConfig = config.profileConfig;
	this->targetHeading = targetHeading;
}

void FollowPathHeading::Initialize() {
	const double startTime = frc::Timer::GetFPGATimestamp();
	
	profileConfig.initialPosition = swerve->getYaw();
	profileConfig.finalPosition = targetHeading;
	profileConfig.startTime = startTime;
	
	profile = std::make_unique<TrapezoidalMotionProfile>(profileConfig);
}

void FollowPathHeading::Execute() {
	const RobotPose2D currentPosition = swerve->getPose();
	const double targetHeading = profile->getPosition_time(frc::Timer::GetFPGATimestamp());
	
	Twist2D velocity = pathFollower->updateUsingCustomHeading(currentPosition, targetHeading);
	swerve->setTargetVelocity(velocity);
}

void FollowPathHeading::End(bool interrupted) {
	swerve->setTargetVelocity(Twist2D(0, 0, 0));
}

bool FollowPathHeading::IsFinished() {
	return pathFollower->hasFinished();
}