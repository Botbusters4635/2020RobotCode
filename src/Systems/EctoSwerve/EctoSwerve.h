//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVE_H
#define BOTBUSTERSREBIRTH_ECTOSWERVE_H

#include <Core/EctoModule/System.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Core/EctoInput/Buttons/EctoButton.h>

#include <Control/SimpleControllerSource.h>
#include <Control/SimpleControllerOutput.h>

#include <Control/Kinematics/Swerve/SwerveKinematics.h>
#include <Control/Kinematics/Swerve/SwerveMotorValues.h>
#include <Control/SwervePIDController/SwervePIDController.h>

#include <Control/Odometry/LinearOdometry.h>
#include <Control/Odometry/ExponentialOdometry.h>

#include "Core/EctoInput/InputManager.h"
#include <Core/MotorHandler/MotorManager.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalOutput.h>
#include <mutex>
#include <thread>
#include "Sensors/EctoDistanceSensor.h"

#include "Math/EctoMath.h"
#include "Math/DataTypes/RobotPose2D.h"

#include "Control/EctoPID/PIDConfig.h"

#include <adi/ADIS16470_IMU.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Notifier.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "GenericSwerveValue.h"

struct EctoSwerveConfig {
	//Angles to which the robot will snap to
	std::vector<double> snappableAngles{0, 2.69, M_PI / 2.0, 0.58, M_PI, -0.58, -M_PI / 2.0, -2.72};
	
	double gearRatio = 8.95;
	double wheelCircumference = 0.10611422637590863 * M_PI;
};

class EctoSwerve : public System {
public:
	explicit EctoSwerve(const EctoSwerveConfig &config);
	
	void initRobot() override;
	
	void updateRobot() override;
	
	void zeroYaw();
	
	double getYaw(bool useZero = true) const;
	
	void setTargetVoltage(const Twist2D &target, const Point2D &centerOfRotation = Point2D(0, 0),
	                      bool transformToFieldOriented = true, bool overrideControllers = false);
	
	void setTargetVelocity(const Twist2D &target, const Point2D &centerOfRotation = Point2D(0, 0),
	                       bool transformToFieldOriented = true, bool overrideControllers = false);
	
	GenericSwerveValue getWheelCurrents() const;
	
	GenericSwerveValue getSteerCurrents() const;
	
	GenericSwerveValue getWheelPIDError() const;
	
	GenericSwerveValue getSteerPIDError() const;
	
	SwerveMotorValues getCurrentMotorStates() const;
	
	/**
	 * Transforms a Twist2D into being field oriented
	 * @param target
	 * @return
	 */
	Twist2D transformToFieldReference(const Twist2D &target) const;
	
	RobotPose2D getPose() const;
	
	Twist2D getVelocity() const;
	
	Twist2D getFieldOrientedVelocity() const;
	
	void updateInitialPositionChooser();

private:
	void setTarget(const Twist2D &target, const Point2D &centerOfRotation = Point2D(0, 0), bool useVelocity = false,
	               bool transformToFieldOriented = true, bool overrideControllers = false);
	
	void initNetworkTables();
	
	void updateNetworkTables();
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	void writeMotors(const SwerveMotorValues &values, bool useVelocity = false);
	
	EctoSwerveConfig config;
	
	//TODO Implement odometry for setTargetPosition
	/**
	 * Kinematics
	 */
	std::unique_ptr<SwerveKinematics> kinematics;
	
	/**
	 * PIDS
	 */
	PIDConfig steerPIDConfig, headingPIDConfig;
	PIDConfig wheelVelocityPID;
	
	std::unique_ptr<SwervePIDController> swervePidController;
	
	std::unique_ptr<EctoPID> targetHeadingController;
	
	/**
	 * Controller Sources / Outputs
	 */
	SwerveControllerSources swervePIDSources;
	SwerveControllerOutput swervePIDOutputs;
	
	SimpleControllerSource headingSource;
	SimpleControllerOutput headingOutput;
	
	/**
	 * Motors
	 */
	MotorManager &motorHandler = MotorManager::getInstance();
	
	std::shared_ptr<EctoMotor> frontRightWheel, frontLeftWheel, backLeftWheel, backRightWheel;
	std::shared_ptr<EctoMotor> frontRightSteer, frontLeftSteer, backLeftSteer, backRightSteer;
	
	double frontRightAnalogOffset, frontLeftAnalogOffset, backLeftAnalogOffset, backRightAnalogOffset;
	
	/**
	 * Heading
	 */
	/**
	 * Defaults to using ADIS16470
	 */
	const bool useNavX = false;
	double headingZero = 0.0;

#ifndef SIMULATION
	std::unique_ptr<frc::ADIS16470_IMU> adis{};
#endif
	
	/**
	 * Targets / Status
	 */
	Twist2D velocityTarget; //Velocity target from setTargetVelocity
	Point2D targetCenterOfRotation;
	bool useVelocityControl = false;
	mutable std::mutex setPointLock;
	
	void updateOdometry(const SwerveMotorValues &motorStates);
	
	void
	updateSetpoints(const SwerveMotorValues &motorStates, const Twist2D &targetVelocity, bool enableVelocityControl,
	                const Point2D &setCenterOfRotation);
	
	/**
	 * Odometry
	 */
	Twist2D currentVelocity;
	LinearOdometry odometry;
	ExponentialOdometry expoOdometry;
	mutable std::mutex odometryLock;
	double odometryPreviousTime = 0;
	
	/**
	 * Odometry sendable chooser
	 */
	frc::SendableChooser<RobotPose2D> initialPositionChooser;
	
	SwerveMotorValues motorStates, lastMotorSetpoint;
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVE_H
