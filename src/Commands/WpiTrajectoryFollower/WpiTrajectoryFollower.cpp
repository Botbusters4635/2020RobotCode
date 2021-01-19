//
// Created by abiel on 2/3/20.
//

#include "WpiTrajectoryFollower.h"

WPITrajectoryFollower::WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn,
                                             const WPITrajectoryFollowerConfig &config,
                                             const frc::Trajectory &trajectory, double finalAngle,
                                             const RobotPose2D &poseToBeginRotation, double multiplier)
		: WPITrajectoryFollower(swerveIn, config, trajectory, finalAngle, multiplier) {
	checkPoseToBeginRotation = true;
	this->poseToBeginRotation = poseToBeginRotation;
}

WPITrajectoryFollower::WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn,
                                             const WPITrajectoryFollowerConfig &config,
                                             const frc::Trajectory &trajectory, double finalAngle, double multiplier)
		: positionVelocityConstraints(units::meters_per_second_t(config.maximumVelocity),
		                              units::meters_per_second_squared_t(config.maximumAcceleration)) {
	this->trajectory = trajectory;
	this->swerve = swerveIn;
	this->config = config;
	
	this->finalAngle = finalAngle;
	
	this->SetName("WPITrajectoryFollower");
	this->multiplier = multiplier;
}

WPITrajectoryFollower::WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn,
                                             const WPITrajectoryFollowerConfig &config,
                                             const frc::Trajectory &trajectory, double finalAngle,
                                             const RobotPose2D &poseToBeginRotation, double multiplier,
                                             const RobotPose2D &poseToApplyMultiplier) : WPITrajectoryFollower(swerveIn,
                                                                                                               config,
                                                                                                               trajectory,
                                                                                                               finalAngle,
                                                                                                               poseToBeginRotation) {
	checkPoseToApplyMultipler = true;
	this->poseToApplyMultiplier = poseToApplyMultiplier;
	targetMultiplier = multiplier;
	this->multiplier = 1.0;
}

WPITrajectoryFollower::WPITrajectoryFollower(const std::shared_ptr<EctoSwerve> &swerveIn,
                                             const WPITrajectoryFollowerConfig &config,
                                             const frc::Trajectory &trajectory, double finalAngle, double multiplier,
                                             const RobotPose2D &poseToApplyMultiplier) : WPITrajectoryFollower(swerveIn,
                                                                                                               config,
                                                                                                               trajectory,
                                                                                                               finalAngle) {
	checkPoseToApplyMultipler = true;
	this->poseToApplyMultiplier = poseToApplyMultiplier;
	targetMultiplier = multiplier;
	this->multiplier = 1.0;
}


void WPITrajectoryFollower::Initialize() {
	updateNotifier = std::make_unique<frc::Notifier>(std::bind(&WPITrajectoryFollower::Update, this));
	
	const frc::Trajectory::State initialState = trajectory.Sample(units::second_t(0));
	const auto targetPose = initialState.pose;
	const auto maxVelocity = initialState.velocity;
	const auto maxAcceleration = initialState.acceleration;
	
	xPositionPID = std::make_unique<frc2::PIDController>(config.positionPIDConfig.p, config.positionPIDConfig.i,
	                                                     config.positionPIDConfig.d, kDt);
	yPositionPID = std::make_unique<frc2::PIDController>(config.positionPIDConfig.p, config.positionPIDConfig.i,
	                                                     config.positionPIDConfig.d, kDt);
	thetaPID = std::make_unique<frc2::PIDController>(config.thetaPIDConfig.p, config.thetaPIDConfig.i,
	                                                 config.thetaPIDConfig.d, kDt);
	thetaPID->EnableContinuousInput(-M_PI, M_PI);
	
	timer.Reset();
	timer.Start();
	
	updateNotifier->StartPeriodic(kDt);
	
	startAngle = swerve->getYaw();
}

void WPITrajectoryFollower::Update() {
	const RobotPose2D currentPosition = swerve->getPose();
	
	const double currentTime = timer.Get();
	
	const frc::Trajectory::State targetState = trajectory.Sample(units::second_t(currentTime));
	const auto targetPose = targetState.pose;
	const auto desiredVelocity = targetState.velocity;
	const auto maxAcceleration = targetState.acceleration;
	
	const auto poseError = targetPose.RelativeTo(
			{units::meter_t(currentPosition.getX()), units::meter_t(currentPosition.getY()),
			 units::radian_t(currentPosition.getHeading().getRadians())});
	
	double xVel = xPositionPID->Calculate(currentPosition.getX(), targetPose.Translation().X().to<double>());
	double yVel = yPositionPID->Calculate(currentPosition.getY(), targetPose.Translation().Y().to<double>());
	
	double thetaVel;
	
	if (checkPoseToBeginRotation) {
		const double distance = std::hypot(currentPosition.getX() - poseToBeginRotation.getX(),
		                                   currentPosition.getY() - poseToBeginRotation.getY());
		
		if (std::abs(distance) <= distanceTrigger) {
			//Rotateeeee
			thetaPID->SetSetpoint(finalAngle);
			std::cout << "ROTATEEEE" << std::endl;
			
			distanceTriggered = true;
		} else if (!distanceTriggered) {
			thetaPID->SetSetpoint(startAngle);
		}
		
		thetaVel = thetaPID->Calculate(currentPosition.getHeading().getRadians());
		
	} else {
		thetaVel = thetaPID->Calculate(currentPosition.getHeading().getRadians(), finalAngle);
	}
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/XVelPIDRaw", xVel);
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/YVelPIDRaw", yVel);
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/thetaPIDRaw", thetaVel);
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/TargetPoseAngle",
	                               targetPose.Rotation().Radians().to<double>());
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/yaw", currentPosition.getHeading().getRadians());
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/finalAngle", finalAngle);
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/DesiredVelocity", desiredVelocity.value());
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/XVel", desiredVelocity.value() * poseError.Rotation().Cos());
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/YVel", desiredVelocity.value() * poseError.Rotation().Sin());
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/ThetaError", thetaPID->GetPositionError());
	
	double angleError = EctoMath::wrapAngle(
			currentPosition.getHeading().getRadians() - targetPose.Rotation().Radians().to<double>());
	
	xVel += 0.3 * desiredVelocity.value() * std::cos(targetPose.Rotation().Radians().to<double>());
	yVel += 0.3 * desiredVelocity.value() * std::sin(targetPose.Rotation().Radians().to<double>());
	
	thetaVel =
			std::abs(thetaVel) > config.maximumAngularVelocity ? std::copysign(config.maximumAngularVelocity, thetaVel)
			                                                   : thetaVel;
	
	const double angularReductionFactor =
			1.0 - ((std::abs(thetaVel) / config.maximumAngularVelocity) * config.angularVelocityReductionFactor);
	
	if (angularReductionFactor > config.angularVelocityReductionFactor) {
		//std::cout << "AAAAAaaa" << std::endl;
	}
	
	//std::cout << "RedFac: " << angularReductionFactor << std::endl;
	
	xVel *= angularReductionFactor;
	yVel *= angularReductionFactor;
	
	//std::cout << "Ang: " << angularReductionFactor << std::endl;
	
	const double maxVel = std::max(std::abs(xVel), std::abs(yVel));
	if (maxVel > config.maximumVelocity) {
		//Normalize velocities
		const double scaleFactor = config.maximumVelocity / maxVel;
		xVel *= scaleFactor;
		yVel *= scaleFactor;
	}
	
	if (checkPoseToApplyMultipler) {
		const double distance = std::hypot(currentPosition.getX() - poseToApplyMultiplier.getX(),
		                                   currentPosition.getY() - poseToApplyMultiplier.getY());
		
		if (std::abs(distance) <= distanceTrigger) {
			multiplier = targetMultiplier;
		}
	}
	
	xVel *= multiplier;
	yVel *= multiplier;
	
	Twist2D outputVel(xVel, yVel, thetaVel);
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/XVel", xVel);
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/YVel", yVel);
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/ThetaVel", thetaVel);
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/XTarget", targetPose.Translation().X().to<double>());
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/YTarget", targetPose.Translation().Y().to<double>());
	
	swerve->setTargetVelocity(outputVel);
}

void WPITrajectoryFollower::End(bool interrupted) {
	updateNotifier->Stop();
	timer.Stop();
	swerve->setTargetVelocity({0, 0, 0});
}

bool WPITrajectoryFollower::IsFinished() {
	const RobotPose2D currentPosition = swerve->getPose();
	
	const frc::Trajectory::State finalState = trajectory.Sample(trajectory.TotalTime());
	
	const double angleError = thetaPID->GetPositionError();
	const double distanceToEnd = std::hypot(currentPosition.getX() - finalState.pose.Translation().X().to<double>(),
	                                        currentPosition.getY() - finalState.pose.Translation().Y().to<double>());
	
	const bool hasReachedDistance = distanceToEnd < config.finishedTreshold;
	const bool hasReachedYaw = std::abs(angleError) < config.finishedAngleTreshold;
	
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/DistanceToEnd", distanceToEnd);
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/hasReachedDistance", hasReachedDistance);
	frc::SmartDashboard::PutNumber("WPITrajectoryFollower/hasReachedYaw", hasReachedYaw);
	
	std::cout << hasReachedDistance << hasReachedYaw << std::endl;
	
	const Twist2D currentVelocity = swerve->getVelocity();
	const auto velocity = std::hypot(currentVelocity.getDx(), currentVelocity.getDy());
	
	std::cout << velocity << std::endl;
	
	//std::cout <<  std::hypot(currentPosition.getX() - finalState.pose.Translation().X().to<double>(), currentPosition.getY() - finalState.pose.Translation().Y().to<double>()) << std::endl;
	
	return (hasReachedDistance) and velocity < config.stoppedVelocity;
}

void WPITrajectoryFollower::setTargetYaw(double target) {
	thetaPID->SetSetpoint(target);
}