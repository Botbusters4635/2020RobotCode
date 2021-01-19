//
// Created by alberto on 03/01/20.
//

#ifndef BOTBUSTERS_REBIRTH_FOLLOWPATH_H
#define BOTBUSTERS_REBIRTH_FOLLOWPATH_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <Control/Path/Path.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Control/PathPlanner/SimplePathPlanner.h"
#include "Control/PathFollowers/Holonomic/PIDHolonomicPathFollower.h"
#include "FollowPathConfig.h"
#include <Control/RateLimiter/Twist2DRateLimiter.h>

class FollowPath : public frc2::CommandHelper<frc2::CommandBase, FollowPath> {
public:
	FollowPath(std::shared_ptr<EctoSwerve> &swerveIn, const Path &path, const FollowPathConfig &config);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<SimplePathPlanner> pathPlanner;
	std::unique_ptr<PIDHolonomicPathFollower> pathFollower;
	
	std::unique_ptr<Twist2DRateLimiter> velocityLimiter;
	
	FollowPathConfig config;
	
	double lastMaxVel = 0;
	double lastRunTime = 0;
};


#endif //BOTBUSTERS_REBIRTH_FOLLOWPATH_H
