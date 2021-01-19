//
// Created by abiel on 2/3/20.
//

#ifndef BOTBUSTERSREBIRTH_WPITRAJECTORYFOLLOWER_H
#define BOTBUSTERSREBIRTH_WPITRAJECTORYFOLLOWER_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc2/Timer.h>
#include <frc/trajectory/Trajectory.h>
#include <vector>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

struct WPITrajectoryFollowerConfig {
	PIDConfig positionPIDConfig;
	PIDConfig thetaPIDConfig;
	
	double finishedTreshold = 0.5;
	double finishedAngleTreshold = 0.1;
	
	double maximumVelocity = 3.0;
	double maximumAcceleration = 4;
	
	double minimumVelocity = 2.4;
	
	double maximumAngularVelocity = 1;
	double maximumAngularAcceleration = 2.5 * M_PI;
	
	double stoppedVelocity = 0.1;
	
	double angularVelocityReductionFactor = 0.75;
};

class WPITrajectoryFollower : public frc2::CommandHelper<frc2::CommandBase, WPITrajectoryFollower> {
public:
	using radians_per_second_squared_t =
	units::compound_unit<units::radians,
			units::inverse<units::squared<units::second>>>;
	
	WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn, const WPITrajectoryFollowerConfig &config,
	                      const frc::Trajectory &trajectory, double finalAngle, const RobotPose2D &poseToBeginRotation,
	                      double multiplier = 1.0);
	
	WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn, const WPITrajectoryFollowerConfig &config,
	                      const frc::Trajectory &trajectory, double finalAngle, const RobotPose2D &poseToBeginRotation,
	                      double multiplier, const RobotPose2D &poseToApplyMultiplier);
	
	WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn, const WPITrajectoryFollowerConfig &config,
	                      const frc::Trajectory &trajectory, double finalAngle, double multiplier = 1.0);
	
	WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn, const WPITrajectoryFollowerConfig &config,
	                      const frc::Trajectory &trajectory, double finalAngle, double multiplier,
	                      const RobotPose2D &poseToApplyMultiplier);
	
	void Initialize() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;
	
	void setTargetYaw(double target);

private:
	void Update();
	
	std::shared_ptr<EctoSwerve> swerve;
	std::unique_ptr<frc2::PIDController> xPositionPID, yPositionPID;
	std::unique_ptr<frc2::PIDController> thetaPID;
	
	std::unique_ptr<frc::Notifier> updateNotifier;
	const frc::TrapezoidProfile<units::meters>::Constraints positionVelocityConstraints;
	
	frc::Trajectory trajectory;
	
	frc::Timer timer;
	
	WPITrajectoryFollowerConfig config;
	
	bool checkPoseToBeginRotation{false};
	RobotPose2D poseToBeginRotation;
	
	double distanceTrigger{0.466};
	bool distanceTriggered{false};
	double startAngle;
	
	double multiplier{1};
	
	double finalAngle;
	constexpr static auto kDt = 0.01_s;
	
	RobotPose2D poseToApplyMultiplier;
	bool checkPoseToApplyMultipler{false};
	double targetMultiplier;
};


#endif //BOTBUSTERSREBIRTH_WPITRAJECTORYFOLLOWER_H
