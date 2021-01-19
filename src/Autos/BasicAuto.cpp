//
// Created by abiel on 2/21/20.
//

#include "BasicAuto.h"

#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc2/command/WaitCommand.h>

BasicAuto::BasicAuto(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<BallFeeder> &ballFeeder,
                     const std::shared_ptr<BallIntake> &ballIntake, const std::shared_ptr<PIDShooter> &shooter) {
	
	SetName("BasicAuto");
	auto startToShoot = frc::TrajectoryUtil::FromPathweaverJson(
			Module::getConfigFileRootDir() + startToShootPath);
	auto goToTrench = frc::TrajectoryUtil::FromPathweaverJson(Module::getConfigFileRootDir() + goToTrenchPath);
	auto grabTrench = frc::TrajectoryUtil::FromPathweaverJson(Module::getConfigFileRootDir() + grabTrenchPath);
	auto shootAfterTrench = frc::TrajectoryUtil::FromPathweaverJson(
			Module::getConfigFileRootDir() + shootAfterTrenchPath);
	
	WPITrajectoryFollowerConfig trajConfig;
	
	trajConfig.positionPIDConfig.p = 2.8;
	trajConfig.positionPIDConfig.i = 0.0;
	trajConfig.positionPIDConfig.d = 0.050;
	
	trajConfig.thetaPIDConfig.p = 1.85;
	trajConfig.thetaPIDConfig.i = 0.0;
	trajConfig.thetaPIDConfig.d = 0.1;
	trajConfig.thetaPIDConfig.f = 0;
	
	trajConfig.maximumVelocity = 4.4;
	trajConfig.maximumAngularVelocity = 2.0;
	trajConfig.angularVelocityReductionFactor = 0.0;
	
	trajConfig.finishedTreshold = 0.5;
	trajConfig.finishedAngleTreshold = 0.2;
	
	WPITrajectoryFollowerConfig grabBallsConfig = trajConfig;
	
	grabBallsConfig.finishedTreshold = .9;
	grabBallsConfig.stoppedVelocity = 0.69;
	
	VisionAlignmentConfig visionAlignHelperConfig;
	visionAlignHelperConfig.vXPIDConfig.p = 1.65;
	visionAlignHelperConfig.vXPIDConfig.d = 0.00150;
	
	visionAlignHelperConfig.vYPIDConfig.p = 1.5;
	visionAlignHelperConfig.vXPIDConfig.d = 0.00150;
	
	visionAlignHelperConfig.thetaPIDConfig.p = 1.8;
	visionAlignHelperConfig.thetaPIDConfig.d = 0.000100;
	
	VisionAlignmentSetpoint alignSetpoint;
	alignSetpoint.desiredDistance = 2.9;
	
	LowerIntakeSetpoint lowerLeftIntake;
	lowerLeftIntake.lowerLeft = true;
	lowerLeftIntake.leftSetpoint = 1.0;
	
	LowerIntakeSetpoint lowerRightIntake;
	lowerRightIntake.lowerRight = true;
	lowerRightIntake.rightSetpoint = 1.0;
	
	LowerIntakeSetpoint offIntake;
	offIntake.leftSetpoint = 1.0;
	
	LowerIntakeSetpoint noIntake;
	
	LowerIntakeSetpoint allOnIntake;
	allOnIntake.lowerLeft = true;
	allOnIntake.lowerRight = true;
	allOnIntake.leftSetpoint = -1.0;
	allOnIntake.rightSetpoint = -1.0;
	
	LowerIntakeSetpoint lowerAll;
	lowerAll.lowerLeft = true;
	lowerAll.lowerRight = true;
	lowerAll.leftSetpoint = 1.0;
	lowerAll.rightSetpoint = 1.0;
	
	AddCommands(
			/**
			 * Align with target and shoot the first 3 balls
			 */
			frc2::ParallelCommandGroup{
					PreloadBall{ballFeeder},
					WPITrajectoryFollower{swerve, trajConfig, startToShoot, M_PI},
					SpinupFlywheel{shooter, 314.34}
			},
			VisionAlign{swerve, visionAlignHelperConfig},
			FeedBalls{ballFeeder, 3},
			
			/**
			 * Go do a trench run and grab 3 balls, afterwards, go to the target and align with it
			 */
			LowerIntake{ballIntake, lowerLeftIntake},
			frc2::ParallelCommandGroup{
					SpinupFlywheel{shooter, 317.34, false},
					frc2::SequentialCommandGroup{
							WPITrajectoryFollower{swerve, grabBallsConfig, goToTrench, -M_PI_2, 0.125,
							                      RobotPose2D(5.632, -1.04, 0)},
							frc2::WaitCommand{units::second_t(0.1)},
							LowerIntake{ballIntake, noIntake},
							WPITrajectoryFollower{swerve, trajConfig, shootAfterTrench, M_PI,
							                      RobotPose2D(5.339, -1.407, 0)},
							LowerIntake{ballIntake, lowerAll},
							VisionAlign{swerve, visionAlignHelperConfig},
					},
					PreloadBall{ballFeeder}
			},
			/**
			 * Shoot all currently held balls, opening and closing the intake
			 */
			LowerIntake{ballIntake, lowerAll},
			FeedBalls{ballFeeder, 1},
			LowerIntake{ballIntake, offIntake},
			FeedBalls{ballFeeder, 1},
			LowerIntake{ballIntake, lowerAll},
			FeedBalls{ballFeeder, 1},
			LowerIntake{ballIntake, offIntake},
			FeedBalls{ballFeeder, 1}
	);
}