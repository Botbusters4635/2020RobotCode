//
// Created by abiel on 1/4/20.
//

#ifndef ECTOCONTROL_PIDHOLONOMICPATHFOLLOWER_H
#define ECTOCONTROL_PIDHOLONOMICPATHFOLLOWER_H

#include "Control/PathPlanner/PathPlanner.h"
#include "Control/PathFollowers/PathFollower.h"
#include "Control/EctoPID/EctoPIDAsynchronous.h"
#include "Control/EctoPID/PIDConfig.h"
#include <memory>

struct PIDHolonomicPathFollowerConfig {
    PIDConfig xPID, yPID, headingPID;
};

class PIDHolonomicPathFollower : public PathFollower {
public:
    PIDHolonomicPathFollower(const PIDHolonomicPathFollowerConfig &config, std::shared_ptr<PathPlanner> plannerIn);

    void setConfigs(const PIDHolonomicPathFollowerConfig &configs);

    Twist2D update(const RobotPose2D &currentPose) override;
    Twist2D updateUsingCustomHeading(const RobotPose2D &currentPose, double targetHeading);

    double getDistanceToPathCompletion() const;
    bool hasFinished() const;

    RobotPose2D getLastTargetPose() const;
private:
    std::shared_ptr<PathPlanner> planner;
    RobotPose2D lastTargetPose;

    EctoPIDAsynchronous xController, yController, headingController;
};


#endif //ECTOCONTROL_PIDHOLONOMICPATHFOLLOWER_H
