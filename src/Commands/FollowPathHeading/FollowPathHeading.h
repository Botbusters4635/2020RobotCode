//
// Created by abiel on 1/4/20.
//

#ifndef BOTBUSTERSREBIRTH_FOLLOWPATHHEADING_H
#define BOTBUSTERSREBIRTH_FOLLOWPATHHEADING_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <Control/PathPlanner/SimplePathPlanner.h>
#include <Control/PathFollowers/Holonomic/PIDHolonomicPathFollower.h>
#include <Control/MotionProfiles/TrapezoidalMotionProfile.h>
#include <Systems/EctoSwerve/EctoSwerve.h>
#include <Control/Path/Path.h>
#include "FollowPathHeadingConfig.h"

/**
 * Follows a path ignoring yaw from the path
 */
class FollowPathHeading : public frc2::CommandHelper<frc2::CommandBase, FollowPathHeading> {
public:
	FollowPathHeading(std::shared_ptr<EctoSwerve> swerve, const Path &path, double targetHeading,
	                  const FollowPathHeadingConfig &config);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<SimplePathPlanner> pathPlanner;
	std::unique_ptr<PIDHolonomicPathFollower> pathFollower;
	std::unique_ptr<TrapezoidalMotionProfile> profile;
	MotionProfileConfig profileConfig;
	double targetHeading;
};


#endif //BOTBUSTERSREBIRTH_FOLLOWPATHHEADING_H
