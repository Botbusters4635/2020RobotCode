//
// Created by abiel on 7/14/20.
//

#include <frc/Timer.h>
#include "EctoTank.h"

EctoTank::EctoTank(const EctoTankConfig &config, const std::shared_ptr<EctoMotor> &leftMotor,
                   const std::shared_ptr<EctoMotor> &rightMotor)
		: System("EctoTank", false) {
	this->config = config;
	this->leftMotor = leftMotor;
	this->rightMotor = rightMotor;
	
	leftMotor->setPIDConfig(config.motorPIDConfig);
	rightMotor->setPIDConfig(config.motorPIDConfig);
	
	diffKinematics = std::make_unique<frc::DifferentialDriveKinematics>(units::meter_t(config.trackWidth));
}

void EctoTank::setTargetVelocity(const Twist2D &target) {
	/**
	 * Convert a Twist2D into -> frc::ChassisSpeeds
	 */
	frc::ChassisSpeeds chassisSpeeds;
	chassisSpeeds.vx = units::meters_per_second_t(target.getDx());
	chassisSpeeds.vy = units::meters_per_second_t(target.getDy());
	chassisSpeeds.omega = units::radians_per_second_t(target.getDtheta());
	
	/**
	 * Use kinematics to turn frc::ChassisSpeeds -> left / right velocities
	 */
	auto wheelSpeeds = diffKinematics->ToWheelSpeeds(chassisSpeeds);
	
	targetVelocityMutex.lock();
	targetLeftVelocity = wheelSpeeds.left.value();
	targetRightVelocity = wheelSpeeds.right.value();
	targetVelocityMutex.unlock();
}

void EctoTank::setTargetVelocity(double leftVelocity, double rightVelocity){
	targetVelocityMutex.lock();
	targetLeftVelocity = leftVelocity;
	targetRightVelocity = rightVelocity;
	targetVelocityMutex.unlock();
}

void EctoTank::setTargetVelocity(units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity) {
	setTargetVelocity(leftVelocity.value() * M_PI / config.wheelDiameter, rightVelocity.value() * M_PI / config.wheelDiameter);
}

RobotPose2D EctoTank::getPose() const {
	return odom.getPose();
}

frc::Pose2d EctoTank::getFRCPose() const {
	auto pose = getPose();
	//I hate units
	return {units::meter_t(pose.getX()), units::meter_t(pose.getY()), frc::Rotation2d(units::radian_t(pose.getHeading().getRadians()))};
}

void EctoTank::initRobot() {
	previousTime = frc::Timer::GetFPGATimestamp();
}

void EctoTank::updateRobot() {
	const double dt = frc::Timer::GetFPGATimestamp() - previousTime;
	
	//Temporary way to get yaw (directly from NT) (only works with sim)
	const double yaw = ntInstance.GetEntry("/TankChassis/imu/yaw").GetDouble(0);
	
	/**
	 * Write setpoints
	 */
	targetVelocityMutex.lock();
	leftMotor->set(targetLeftVelocity, MotorControlMode::Velocity);
	rightMotor->set(targetRightVelocity, MotorControlMode::Velocity);
	targetVelocityMutex.unlock();
	
	/**
	 * Read velocities and calculate odometry
	 */
	frc::DifferentialDriveWheelSpeeds driveWheelSpeeds;
	driveWheelSpeeds.left = units::meters_per_second_t(leftMotor->getQuadVelocity() * config.wheelDiameter * 0.5);
	driveWheelSpeeds.right = units::meters_per_second_t(rightMotor->getQuadVelocity() * config.wheelDiameter * 0.5);
	
	frc::ChassisSpeeds currentVelocityFrc = diffKinematics->ToChassisSpeeds(driveWheelSpeeds);
	
	//Convert frc::ChassisSpeeds to (our) Twist2D
	Twist2D currentVelocity(currentVelocityFrc.vx.value(), currentVelocityFrc.vy.value(),
	                        yaw);
	
	odom.updateOdometry_velocity(currentVelocity, dt);
	
	previousTime = frc::Timer::GetFPGATimestamp();
}

void EctoTank::resetOdometry(const RobotPose2D &newVal){
	odom.resetPosition(newVal);
}