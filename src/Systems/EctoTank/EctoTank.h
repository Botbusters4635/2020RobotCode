//
// Created by abiel on 7/14/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOTANK_H
#define BOTBUSTERSREBIRTH_ECTOTANK_H

#include <Core/EctoModule/System.h>
#include <Math/DataTypes/Twist2D.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <Control/Odometry/ExponentialOdometry.h>
#include <Control/EctoPID/PIDConfig.h>

#include <networktables/NetworkTableInstance.h>

#include <frc/geometry/Pose2d.h>

struct EctoTankConfig {
	double wheelDiameter = 1.0;
	
	double trackWidth = 1;
	
	PIDConfig motorPIDConfig;
};

class EctoTank : public System {
public:
	EctoTank(const EctoTankConfig &config, const std::shared_ptr<EctoMotor> &leftMotor, const std
	::shared_ptr<EctoMotor> &rightMotor);
	
	void initRobot() override;
	
	void updateRobot() override;
	
	void setTargetVelocity(const Twist2D &target);
	
	void setTargetVelocity(double leftVelocity, double rightVelocity);
	
	void setTargetVelocity(units::meters_per_second_t leftVelocity, units::meters_per_second_t rightVelocity);

	void resetOdometry(const RobotPose2D &newVal);
	
	RobotPose2D getPose() const;
	
	frc::Pose2d getFRCPose() const;
	
	std::unique_ptr<frc::DifferentialDriveKinematics> diffKinematics;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	
	EctoTankConfig config;
	
	std::shared_ptr<EctoMotor> leftMotor, rightMotor;
	
	std::mutex targetVelocityMutex;
	double targetLeftVelocity, targetRightVelocity;
	
	ExponentialOdometry odom;
	
	double previousTime;
};


#endif //BOTBUSTERSREBIRTH_ECTOTANK_H
