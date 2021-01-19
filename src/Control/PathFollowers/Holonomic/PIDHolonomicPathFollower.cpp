//
// Created by abiel on 1/4/20.
//

#include "Control/PathFollowers/Holonomic/PIDHolonomicPathFollower.h"

PIDHolonomicPathFollower::PIDHolonomicPathFollower(const PIDHolonomicPathFollowerConfig &config,
                                                   std::shared_ptr<PathPlanner> plannerIn) :
                                                   planner(plannerIn),
                                                   xController(config.xPID),
                                                   yController(config.yPID),
                                                   headingController(config.headingPID) {
    ;
}

void PIDHolonomicPathFollower::setConfigs(const PIDHolonomicPathFollowerConfig &configs) {
    xController.setConfig(configs.xPID);
    yController.setConfig(configs.yPID);
    headingController.setConfig(configs.headingPID);
}

Twist2D PIDHolonomicPathFollower::update(const RobotPose2D &currentPose) {
    lastTargetPose = planner->update(currentPose);

    Twist2D output;
    xController.setSetpoint(lastTargetPose.getX());
    yController.setSetpoint(lastTargetPose.getY());
    headingController.setSetpoint(lastTargetPose.getHeading().getRadians());

    output.setDx(xController.update(currentPose.getX()));
    output.setDy(yController.update(currentPose.getY()));
    output.setDtheta(headingController.update(currentPose.getHeading().getRadians()));

    return output;
}

Twist2D PIDHolonomicPathFollower::updateUsingCustomHeading(const RobotPose2D &currentPose, double targetHeading) {
    lastTargetPose = planner->update(currentPose);

    Twist2D output;
    xController.setSetpoint(lastTargetPose.getX());
    yController.setSetpoint(lastTargetPose.getY());
    headingController.setSetpoint(targetHeading);

    output.setDx(xController.update(currentPose.getX()));
    output.setDy(yController.update(currentPose.getY()));
    output.setDtheta(headingController.update(currentPose.getHeading().getRadians()));

    return output;
}

double PIDHolonomicPathFollower::getDistanceToPathCompletion() const {
    return planner->getDistanceToPathCompletion();
}

bool PIDHolonomicPathFollower::hasFinished() const {
    return planner->hasFinished();
}

RobotPose2D PIDHolonomicPathFollower::getLastTargetPose() const {
    return lastTargetPose;
}