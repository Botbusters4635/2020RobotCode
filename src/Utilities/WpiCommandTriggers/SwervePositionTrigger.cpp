//
// Created by abiel on 2/22/20.
//


#include "SwervePositionTrigger.h"

SwervePositionTrigger::SwervePositionTrigger(std::shared_ptr<const EctoSwerve> &swerve, const RobotPose2D &triggerPose,
                                             double distanceTolerance) {
	this->swerve = swerve;
	this->distanceTolerance = distanceTolerance;
	
	if (distanceTolerance <= 0.0) {
		throw std::logic_error("Invalid distance tolerance given to SwervePositionTrigger");
	}
	
	this->triggerPose = triggerPose;
}

bool SwervePositionTrigger::get() {
	return std::abs(RobotPose2D::getDistanceBetweenPoints(swerve->getPose(), triggerPose)) >=
	       distanceTolerance; //std::abs probably isn't needed, used just in case
}