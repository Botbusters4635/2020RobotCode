//
// Created by abiel on 2/19/20.
//

#ifndef BOTBUSTERSREBIRTH_WPITRAJECTORYVISIONFOLLOWER_H
#define BOTBUSTERSREBIRTH_WPITRAJECTORYVISIONFOLLOWER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Core/VisionManager/VisionManager.h"

#include "Commands/WpiTrajectoryFollower/WpiTrajectoryFollower.h"

/**
 * Follows a path while trying to align yaw with a vision target
 */
class WPITrajectoryVisionFollower : public frc2::CommandHelper<frc2::CommandBase, WPITrajectoryVisionFollower> {
public:
	WPITrajectoryVisionFollower(const std::shared_ptr<EctoSwerve> &swerveIn, const WPITrajectoryFollowerConfig config,
	                            const frc::Trajectory &trajectory, double defaultAngle);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	
	VisionManager &manager = VisionManager::getInstance();
	
	std::unique_ptr<WPITrajectoryFollower> trajectoryFollower;
};


#endif //BOTBUSTERSREBIRTH_WPITRAJECTORYVISIONFOLLOWER_H
