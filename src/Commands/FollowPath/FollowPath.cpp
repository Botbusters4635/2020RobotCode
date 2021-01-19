//
// Created by alberto on 03/01/20.
//

#include "FollowPath.h"
#include <iostream>
#include <utility>

FollowPath::FollowPath(std::shared_ptr<EctoSwerve> &swerveIn, const Path &path, const FollowPathConfig &config) {
	this->swerve = swerveIn;
	this->config = config;
	
	lastMaxVel = config.maxVelocity;
	
	pathPlanner = std::make_shared<SimplePathPlanner>(path, config.lookaheadDistance, config.finishThreshold);
	pathFollower = std::make_unique<PIDHolonomicPathFollower>(config.pathFollowerConfig, pathPlanner);
	
	velocityLimiter = std::make_unique<Twist2DRateLimiter>(config.maxAcceleration);
}

void FollowPath::Initialize() {
	lastRunTime = frc::Timer::GetFPGATimestamp();
}

void FollowPath::Execute() {
	const RobotPose2D currentPosition = swerve->getPose();
	
	Twist2D velocity = pathFollower->update(currentPosition);
	
	std::cout << pathFollower->getLastTargetPose() << std::endl;
	
	double maxVel = config.maxVelocity;
	
	//Decelerate when needed
	const double distanceToFinalPoint = pathFollower->getDistanceToPathCompletion();
	const double velocityMag = std::hypot(velocity.getDx(), velocity.getDy());
	const double distanceNeededToDeaccelerate = std::pow(velocityMag, 2.0) / (2.0 * config.maxDeacceleration);
	std::cout << "DIST: " << distanceNeededToDeaccelerate << ", FINAL: " << distanceToFinalPoint << std::endl;
	if (distanceToFinalPoint < distanceNeededToDeaccelerate) {
		const double dtDeaccel = frc::Timer::GetFPGATimestamp() - lastRunTime;
		std::cout << "DT: " << dtDeaccel << std::endl;
		maxVel = lastMaxVel - (config.maxDeacceleration * dtDeaccel);
		
		std::cout << "MAXVEL: " << maxVel << std::endl;
		
		lastMaxVel = maxVel;
		
		maxVel = std::max(maxVel, config.minVelocity);
	}
	
	//Limit max velocity, uses the largest value and scales all velocities
	const double maxSetVelocity = std::max(std::abs(velocity.getDx()), std::abs(velocity.getDy()));
	if (std::abs(maxSetVelocity) > maxVel) {
		const double scaleFactor = maxVel / maxSetVelocity;
		velocity.setDx(velocity.getDx() * scaleFactor);
		velocity.setDy(velocity.getDy() * scaleFactor);
	}
	
	velocityLimiter->set(velocity);
	
	swerve->setTargetVelocity(velocityLimiter->get(frc::Timer::GetFPGATimestamp() - lastRunTime));
	
	lastRunTime = frc::Timer::GetFPGATimestamp();
}

void FollowPath::End(bool interrupted) {
	swerve->setTargetVelocity(Twist2D(0, 0, 0));
}

bool FollowPath::IsFinished() {
	return pathFollower->hasFinished();
}


