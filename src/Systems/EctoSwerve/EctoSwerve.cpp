//
// Created by abiel on 1/2/20.
//

#include "EctoSwerve.h"
#include "frc/smartdashboard/SmartDashboard.h"

EctoSwerve::EctoSwerve(const EctoSwerveConfig &config) : System("EctoSwerve") {
	/*
	 * Kinematics init
	 */
	const double length = settings->getNumber("Swerve", "Length");
	const double width = settings->getNumber("Swerve", "Width");
	kinematics = std::make_unique<SwerveKinematics>(length, width);
	
	/**
	 * Odometry chooser init
	 */
	RobotPose2D leftPosition(-1, 0, 0);
	RobotPose2D centerPosition(2.7, -0.405, M_PI);
	initialPositionChooser.AddOption("LeftPosition", leftPosition);
	initialPositionChooser.SetDefaultOption("CenterPosition", centerPosition);
	
	frc::SmartDashboard::PutData("EctoSwerve/InitialPositionChooser", &initialPositionChooser);
	
	/*
	 * Heading PID Controller Config
	 */
	headingPIDConfig.continous = true;
	headingPIDConfig.maxInput = M_PI;
	headingPIDConfig.minInput = -M_PI;
	headingPIDConfig.clamped = true;
	headingPIDConfig.maxOutput = 4.0 * M_PI;
	headingPIDConfig.minOutput = -4.0 * M_PI;
	headingPIDConfig.deadband = 0.002;
	headingPIDConfig.p = 0.9;
	headingPIDConfig.i = 0.0;
	headingPIDConfig.d = 0.003;
	
	targetHeadingController = std::make_unique<EctoPID>(headingSource, headingOutput, headingPIDConfig);
	
	/*
	 * Spark init
	 */
	frontLeftWheel = motorHandler.getMotor("front_left_wheel");
	frontRightWheel = motorHandler.getMotor("front_right_wheel");
	backLeftWheel = motorHandler.getMotor("back_left_wheel");
	backRightWheel = motorHandler.getMotor("back_right_wheel");
	
	frontLeftSteer = motorHandler.getMotor("front_left_steer");
	frontRightSteer = motorHandler.getMotor("front_right_steer");
	backLeftSteer = motorHandler.getMotor("back_left_steer");
	backRightSteer = motorHandler.getMotor("back_right_steer");
	
	/**
	 * Feedback and source mode
	 */
	frontRightSteer->setDefaultFeedbackMode(MotorFeedbackMode::Potentiometer);
	frontLeftSteer->setDefaultFeedbackMode(MotorFeedbackMode::Potentiometer);
	backRightSteer->setDefaultFeedbackMode(MotorFeedbackMode::Potentiometer);
	backLeftSteer->setDefaultFeedbackMode(MotorFeedbackMode::Potentiometer);
	
	frontRightSteer->setControllerSourceMode(MotorControlMode::Position);
	frontLeftSteer->setControllerSourceMode(MotorControlMode::Position);
	backRightSteer->setControllerSourceMode(MotorControlMode::Position);
	backLeftSteer->setControllerSourceMode(MotorControlMode::Position);
	
	frontRightWheel->setDefaultFeedbackMode(MotorFeedbackMode::QuadEncoder);
	frontLeftWheel->setDefaultFeedbackMode(MotorFeedbackMode::QuadEncoder);
	backRightWheel->setDefaultFeedbackMode(MotorFeedbackMode::QuadEncoder);
	backLeftWheel->setDefaultFeedbackMode(MotorFeedbackMode::QuadEncoder);
	
	frontRightWheel->setQuadAsClosedLoopSource();
	frontLeftWheel->setQuadAsClosedLoopSource();
	backRightWheel->setQuadAsClosedLoopSource();
	backLeftWheel->setQuadAsClosedLoopSource();
	
	/**
	 * Inverts
	 */
	frontRightSteer->invert(true);
	frontLeftSteer->invert(true);
	backRightSteer->invert(true);
	backLeftSteer->invert(true);
	
	frontRightWheel->invert(false);
	backRightWheel->invert(false);
	frontLeftWheel->invert(true);
	backLeftWheel->invert(true);
	
	/**
	 * Ramp Rates
	 */
	const double wheelRampRate = 0.09;
	frontRightWheel->setOpenLoopRampRate(wheelRampRate);
	frontLeftWheel->setOpenLoopRampRate(wheelRampRate);
	backRightWheel->setOpenLoopRampRate(wheelRampRate);
	backLeftWheel->setOpenLoopRampRate(wheelRampRate);
	
	frontRightWheel->setClosedLoopRampRate(wheelRampRate);
	frontLeftWheel->setClosedLoopRampRate(wheelRampRate);
	backRightWheel->setClosedLoopRampRate(wheelRampRate);
	backLeftWheel->setClosedLoopRampRate(wheelRampRate);
	
	const double steerRampRate = .05;
	frontRightSteer->setOpenLoopRampRate(steerRampRate);
	frontLeftSteer->setOpenLoopRampRate(steerRampRate);
	backRightSteer->setOpenLoopRampRate(steerRampRate);
	backLeftSteer->setOpenLoopRampRate(steerRampRate);
	
	/**
	 * Brakes
	 */
	const bool wheelBrake = true;
	frontRightWheel->enableBrakingOnIdle(wheelBrake);
	frontLeftWheel->enableBrakingOnIdle(wheelBrake);
	backRightWheel->enableBrakingOnIdle(wheelBrake);
	backLeftWheel->enableBrakingOnIdle(wheelBrake);
	
	const bool steerBrake = true;
	frontRightSteer->enableBrakingOnIdle(steerBrake);
	frontLeftSteer->enableBrakingOnIdle(steerBrake);
	backLeftSteer->enableBrakingOnIdle(steerBrake);
	backRightSteer->enableBrakingOnIdle(steerBrake);
	
	/**
	 * Current limits
	 */
	const double wheelCurrentLimit = 30;
	frontRightWheel->setMotorCurrentLimit(wheelCurrentLimit);
	frontRightWheel->enableCurrentLimit(true);
	
	frontLeftWheel->setMotorCurrentLimit(wheelCurrentLimit);
	frontLeftWheel->enableCurrentLimit(true);
	
	backRightWheel->setMotorCurrentLimit(wheelCurrentLimit);
	backRightWheel->enableCurrentLimit(true);
	
	backLeftWheel->setMotorCurrentLimit(wheelCurrentLimit);
	backLeftWheel->enableCurrentLimit(true);
	
	const double steerCurrentLimit = 30;
	frontRightSteer->setMotorCurrentLimit(steerCurrentLimit);
	frontRightSteer->enableCurrentLimit(true);
	
	frontLeftSteer->setMotorCurrentLimit(steerCurrentLimit);
	frontLeftSteer->enableCurrentLimit(true);
	
	backRightSteer->setMotorCurrentLimit(steerCurrentLimit);
	backRightSteer->enableCurrentLimit(true);
	
	backLeftSteer->setMotorCurrentLimit(steerCurrentLimit);
	backLeftSteer->enableCurrentLimit(true);
	
	/**
	 * Analog position conversion factor
	 */
	frontRightSteer->setAnalogPositionConversionFactor(2 * M_PI / 3.335);
	frontLeftSteer->setAnalogPositionConversionFactor(2 * M_PI / 3.335);
	backRightSteer->setAnalogPositionConversionFactor(2 * M_PI / 3.335);
	backLeftSteer->setAnalogPositionConversionFactor(2 * M_PI / 3.335);
	
	/**
	 * Analog offsets
	 */
//	frontLeftAnalogOffset = 3.623;
//	frontRightAnalogOffset = 5.201;
//
//	backLeftAnalogOffset = 2.885;
//	backRightAnalogOffset = 0.923;
	frontRightAnalogOffset = 1.451;
	frontLeftAnalogOffset = 4.131;
	
	backRightAnalogOffset = 3.476;
	backLeftAnalogOffset = 0.645;
	
	frontLeftSteer->setAnalogSensorOffset(frontLeftAnalogOffset);
	frontRightSteer->setAnalogSensorOffset(frontRightAnalogOffset);
	
	backLeftSteer->setAnalogSensorOffset(backLeftAnalogOffset);
	backRightSteer->setAnalogSensorOffset(backRightAnalogOffset);
	
	/**
	 * Steer PID init
	 */
	steerPIDConfig.p = 0.35;
	steerPIDConfig.d = 0.0015;
	steerPIDConfig.minInput = -M_PI;
	steerPIDConfig.maxInput = M_PI;
	steerPIDConfig.continous = true;
	steerPIDConfig.deadband = 0.0;
	
	swervePIDSources.frontLeftSteer = frontLeftSteer;
	swervePIDSources.frontRightSteer = frontRightSteer;
	swervePIDSources.backLeftSteer = backLeftSteer;
	swervePIDSources.backRightSteer = backRightSteer;
	
	swervePIDOutputs.frontLeftSteer = frontLeftSteer;
	swervePIDOutputs.frontRightSteer = frontRightSteer;
	swervePIDOutputs.backLeftSteer = backLeftSteer;
	swervePIDOutputs.backRightSteer = backRightSteer;
	
	swervePidController = std::make_unique<SwervePIDController>(swervePIDSources, swervePIDOutputs, steerPIDConfig);
	swervePidController->start();
	
	//TODO Tune PIDs
	wheelVelocityPID.p = 0.000017;
	wheelVelocityPID.d = 0.0;
	wheelVelocityPID.f = 0.000176;
	
	frontLeftWheel->setPIDConfig(wheelVelocityPID, 0);
	frontRightWheel->setPIDConfig(wheelVelocityPID, 0);
	backLeftWheel->setPIDConfig(wheelVelocityPID, 0);
	backRightWheel->setPIDConfig(wheelVelocityPID, 0);
	
	table = ntInstance.GetTable("EctoSwerve");

#ifndef SIMULATION
	/**
	* Gyro init
	*/
	adis = std::make_unique<frc::ADIS16470_IMU>(frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0,
	                                            frc::ADIS16470CalibrationTime::_4s);
	//adis.Calibrate();
	
	adis->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
#endif
	
	
	zeroYaw();
	initNetworkTables();
}

void EctoSwerve::initNetworkTables() {
	table->GetEntry("EctoSwerve/Heading/P").SetDouble(headingPIDConfig.p);
	table->GetEntry("EctoSwerve/Heading/I").SetDouble(headingPIDConfig.i);
	table->GetEntry("EctoSwerve/Heading/D").SetDouble(headingPIDConfig.d);
	
	table->GetEntry("EctoSwerve/Steer/P").SetDouble(steerPIDConfig.p);
	table->GetEntry("EctoSwerve/Steer/I").SetDouble(steerPIDConfig.i);
	table->GetEntry("EctoSwerve/Steer/D").SetDouble(steerPIDConfig.d);
	
	table->GetEntry("EctoSwerve/WheelVelocity/P").SetDouble(wheelVelocityPID.p);
	table->GetEntry("EctoSwerve/WheelVelocity/I").SetDouble(wheelVelocityPID.i);
	table->GetEntry("EctoSwerve/WheelVelocity/D").SetDouble(wheelVelocityPID.d);
	table->GetEntry("EctoSwerve/WheelVelocity/F").SetDouble(wheelVelocityPID.f);
	
	table->GetEntry("EctoSwerve/WheelVelocity/MinInput").SetDouble(wheelVelocityPID.minInput);
	table->GetEntry("EctoSwerve/WheelVelocity/MaxInput").SetDouble(wheelVelocityPID.maxInput);
}

void EctoSwerve::updateNetworkTables() {
	table->GetEntry("EctoSwerve/CurrentHeading").SetDouble(getYaw());
	
	PIDConfig newSteerConfig = steerPIDConfig;
	newSteerConfig.p = table->GetEntry("EctoSwerve/Steer/P").GetDouble(steerPIDConfig.p);
	newSteerConfig.i = table->GetEntry("EctoSwerve/Steer/I").GetDouble(steerPIDConfig.i);
	newSteerConfig.d = table->GetEntry("EctoSwerve/Steer/D").GetDouble(steerPIDConfig.d);
	
	if (newSteerConfig != steerPIDConfig) {
		swervePidController->setConfig(newSteerConfig);
		steerPIDConfig = newSteerConfig;
	}
	
	PIDConfig newHeadingConfig = headingPIDConfig;
	newHeadingConfig.p = table->GetEntry("EctoSwerve/Heading/P").GetDouble(headingPIDConfig.p);
	newHeadingConfig.i = table->GetEntry("EctoSwerve/Heading/I").GetDouble(headingPIDConfig.i);
	newHeadingConfig.d = table->GetEntry("EctoSwerve/Heading/D").GetDouble(headingPIDConfig.d);
	
	if (newHeadingConfig != headingPIDConfig) {
		targetHeadingController->setConfig(newHeadingConfig);
		headingPIDConfig = newHeadingConfig;
	}
	
	PIDConfig newWheelVelocityPID = wheelVelocityPID;
	newWheelVelocityPID.p = table->GetEntry("EctoSwerve/WheelVelocity/P").GetDouble(wheelVelocityPID.p);
	newWheelVelocityPID.i = table->GetEntry("EctoSwerve/WheelVelocity/I").GetDouble(wheelVelocityPID.i);
	newWheelVelocityPID.d = table->GetEntry("EctoSwerve/WheelVelocity/D").GetDouble(wheelVelocityPID.d);
	newWheelVelocityPID.f = table->GetEntry("EctoSwerve/WheelVelocity/F").GetDouble(wheelVelocityPID.f);
	newWheelVelocityPID.minInput = table->GetEntry("EctoSwerve/WheelVelocity/MinInput").GetDouble(
			wheelVelocityPID.minInput);
	newWheelVelocityPID.maxInput = table->GetEntry("EctoSwerve/WheelVelocity/MaxOutput").GetDouble(
			wheelVelocityPID.maxInput);
	
	if (newWheelVelocityPID != wheelVelocityPID) {
		frontLeftWheel->setPIDConfig(newWheelVelocityPID, 0);
		frontRightWheel->setPIDConfig(newWheelVelocityPID, 0);
		backLeftWheel->setPIDConfig(newWheelVelocityPID, 0);
		backRightWheel->setPIDConfig(newWheelVelocityPID, 0);
		wheelVelocityPID = newWheelVelocityPID;
	}
	
	RobotPose2D currentPose;
	Twist2D currentVelocity;
	
	odometryLock.lock();
	currentVelocity = this->currentVelocity;
	odometryLock.unlock();
	
	currentPose = getPose();
	
	table->GetEntry("EctoSwerve/Odometry/Velocity/dX").SetDouble(currentVelocity.getDx());
	table->GetEntry("EctoSwerve/Odometry/Velocity/dY").SetDouble(currentVelocity.getDy());
	table->GetEntry("EctoSwerve/Odometry/Velocity/dTheta").SetDouble(currentVelocity.getDtheta());
	
	table->GetEntry("EctoSwerve/Odometry/Pose/X").SetDouble(currentPose.getPoint().getX());
	table->GetEntry("EctoSwerve/Odometry/Pose/Y").SetDouble(currentPose.getPoint().getY());
	table->GetEntry("EctoSwerve/Odometry/Pose/Heading").SetDouble(currentPose.getHeading().getRadians());
	
	table->GetEntry("EctoSwerve/Analog/FrontLeft").SetDouble(frontLeftSteer->getRawAnalogPosition());
	table->GetEntry("EctoSwerve/Analog/FrontRight").SetDouble(frontRightSteer->getRawAnalogPosition());
	table->GetEntry("EctoSwerve/Analog/BackLeft").SetDouble(backLeftSteer->getRawAnalogPosition());
	table->GetEntry("EctoSwerve/Analog/BackRight").SetDouble(backRightSteer->getRawAnalogPosition());
}

void EctoSwerve::setTargetVelocity(const Twist2D &target, const Point2D &centerOfRotation,
                                   bool transformToFieldOriented, bool overrideControllers) {
	setTarget(target, centerOfRotation, true, transformToFieldOriented, overrideControllers);
}

void EctoSwerve::setTargetVoltage(const Twist2D &target, const Point2D &centerOfRotation, bool transformToFieldOriented,
                                  bool overrideControllers) {
	setTarget(target, centerOfRotation, false, transformToFieldOriented, overrideControllers);
}

void EctoSwerve::setTarget(const Twist2D &target, const Point2D &centerOfRotation,
                           bool useVelocity, bool transformToFieldOriented, bool overrideControllers) {
	this->targetCenterOfRotation = centerOfRotation;
	this->velocityTarget = transformToFieldOriented ? transformToFieldReference(target) : target;
	useVelocityControl = useVelocity;
}

Twist2D EctoSwerve::transformToFieldReference(const Twist2D &target) const {
	const double yaw = getYaw();
	const double yawCos = std::cos(yaw);
	const double yawSin = std::sin(yaw);
	
	Twist2D output;
	
	const double temp = target.getDx() * yawCos + target.getDy() * yawSin;
	output.setDy(target.getDy() * yawCos - target.getDx() * yawSin);
	output.setDx(temp);
	output.setDtheta(target.getDtheta());
	
	return output;
}

SwerveMotorValues EctoSwerve::getCurrentMotorStates() const {
	SwerveMotorValues values;
	
	//Convert from rad / sec -> m/s
	values.topLeft.wheelVelocity =
			(frontLeftWheel->getQuadVelocity() / (2.0 * M_PI)) / config.gearRatio * config.wheelCircumference;
	values.topLeft.wheelAngle = -frontLeftSteer->getPotPosition();
	values.topLeft.wheelAngularVelocity = -frontLeftSteer->getPotVelocity();
	
	values.topRight.wheelVelocity =
			(frontRightWheel->getQuadVelocity() / (2.0 * M_PI)) / config.gearRatio * config.wheelCircumference;
	values.topRight.wheelAngle = -frontRightSteer->getPotPosition();
	values.topRight.wheelAngularVelocity = -frontRightSteer->getPotVelocity();
	
	values.backLeft.wheelVelocity =
			(backLeftWheel->getQuadVelocity() / (2.0 * M_PI)) / config.gearRatio * config.wheelCircumference;
	values.backLeft.wheelAngle = -backLeftSteer->getPotPosition();
	values.backLeft.wheelAngularVelocity = -backLeftSteer->getPotVelocity();
	
	values.backRight.wheelVelocity =
			(backRightWheel->getQuadVelocity() / (2.0 * M_PI)) / config.gearRatio * config.wheelCircumference;
	values.backRight.wheelAngle = -backRightSteer->getPotPosition();
	values.backRight.wheelAngularVelocity = -backRightSteer->getPotVelocity();
	
	table->GetEntry("EctoSwerve/WheelVelocity/TopLeft").SetDouble(values.topLeft.wheelVelocity);
	
	return values;
}

void EctoSwerve::writeMotors(const SwerveMotorValues &values, bool useVelocity) {
	if (useVelocity) {
		//M/S -> rot/s
		//log->info("{}", (values.backRight.wheelVelocity / config.wheelCircumference * config.gearRatio * (2.0 * M_PI)));
		//frontLeftWheel->set(values.topLeft.wheelVelocity * (2.0 * M_PI) * config.gearRatio, EctoControlMode::Velocity);
		//frontRightWheel->set(values.topRight.wheelVelocity * (2.0 * M_PI) * config.gearRatio, EctoControlMode::Velocity);
		// backLeftWheel->set(values.backLeft.wheelVelocity * (2.0 * M_PI) * config.gearRatio, EctoControlMode::Velocity);
		frontLeftWheel->set(values.topLeft.wheelVelocity / config.wheelCircumference * config.gearRatio * (2.0 * M_PI),
		                    MotorControlMode::Velocity);
		frontRightWheel->set(
				values.topRight.wheelVelocity / config.wheelCircumference * config.gearRatio * (2.0 * M_PI),
				MotorControlMode::Velocity);
		backRightWheel->set(
				values.backRight.wheelVelocity / config.wheelCircumference * config.gearRatio * (2.0 * M_PI),
				MotorControlMode::Velocity);
		backLeftWheel->set(values.backLeft.wheelVelocity / config.wheelCircumference * config.gearRatio * (2.0 * M_PI),
		                   MotorControlMode::Velocity);
	} else {
		frontLeftWheel->set(values.topLeft.wheelVelocity, MotorControlMode::Percent);
		frontRightWheel->set(values.topRight.wheelVelocity, MotorControlMode::Percent);
		backLeftWheel->set(values.backLeft.wheelVelocity, MotorControlMode::Percent);
		backRightWheel->set(values.backRight.wheelVelocity, MotorControlMode::Percent);
	}
	
	swervePidController->set(values.topLeft.wheelAngle, values.topRight.wheelAngle, values.backLeft.wheelAngle,
	                         values.backRight.wheelAngle);
}

void EctoSwerve::initRobot() {
	;
}

void EctoSwerve::updateRobot() {
	updateNetworkTables();
	
	motorStates = getCurrentMotorStates();
	
	updateOdometry(motorStates);
	
	//Cache setpoints
	setPointLock.lock();
	const Twist2D cacheSetVelocity = velocityTarget;
	const auto cacheTargetCenterOfRotation = targetCenterOfRotation;
	const bool cacheVelocityControl = useVelocityControl;
	setPointLock.unlock();
	
	updateSetpoints(motorStates, cacheSetVelocity, cacheVelocityControl, cacheTargetCenterOfRotation);
}

void EctoSwerve::zeroYaw() {
	/**
	 * NavX Reset
	 */
	headingZero = getYaw(false);
}


double EctoSwerve::getYaw(bool useZero) const {
	double yaw = 0;

#ifndef SIMULATION
	yaw = adis->GetAngle() * (M_PI / 180.0); //Use floats for now until we switch over to the ADIS
	yaw = EctoMath::wrapAngle(yaw);
	//Wrap angle
	//yaw = std::fmod(yaw, 360.0);
	
	//Use -180 to 180
	//yaw -= 180.0;
#endif
	
	if (useZero) {
		yaw -= headingZero;
	}
	
	double beforeWrap = yaw;
	yaw = EctoMath::wrapAngle(yaw);
	
	return yaw;
}

RobotPose2D EctoSwerve::getPose() const {
	//std::lock_guard<std::mutex> lock(odometryLock);
	RobotPose2D pose = odometry.getPose();
	pose.setHeading(Rotation2D(getYaw()));
	return pose;
}

Twist2D EctoSwerve::getVelocity() const {
	std::lock_guard<std::mutex> lock(odometryLock);
	Twist2D velocity = currentVelocity;
	return velocity;
}

Twist2D EctoSwerve::getFieldOrientedVelocity() const {
	const Twist2D currentVelocity = getVelocity();
	return transformToFieldReference(currentVelocity);
}

void EctoSwerve::updateSetpoints(const SwerveMotorValues &motorStates, const Twist2D &targetVelocity,
                                 bool enableVelocityControl, const Point2D &setCenterOfRotation) {
	table->GetEntry("EctoSwerve/TargetdX").SetDouble(targetVelocity.getDx());
	table->GetEntry("EctoSwerve/TargetdY").SetDouble(targetVelocity.getDy());
	table->GetEntry("EctoSwerve/TargetdTheta").SetDouble(targetVelocity.getDtheta());
	table->GetEntry("EctoSwerve/UseVelocityMode").SetBoolean(enableVelocityControl);
	
	SwerveMotorValues motorValues = kinematics->calculateInverseKinematics(targetVelocity, motorStates,
	                                                                       setCenterOfRotation);
	
	lastMotorSetpoint = motorValues;
	
	writeMotors(motorValues, enableVelocityControl);
}

void EctoSwerve::updateOdometry(const SwerveMotorValues &motorStates) {
	const double dt = frc::Timer::GetFPGATimestamp() - odometryPreviousTime;
	
	//Cache forward kinematics and then lock
	Twist2D currentVelocityCalc = kinematics->calculateForwardKinematics(motorStates);
	currentVelocityCalc.setDtheta(getYaw());
	
	odometryLock.lock();
	this->currentVelocity = currentVelocityCalc;
	odometry.updateOdometry_velocity(currentVelocity, dt);
	expoOdometry.updateOdometry_velocity(currentVelocity, dt);
	odometryLock.unlock();
	
	table->GetEntry("EctoSwerve/LinearOdometry/X").SetDouble(odometry.getPose().getX());
	table->GetEntry("EctoSwerve/LinearOdometry/Y").SetDouble(odometry.getPose().getY());
	
	table->GetEntry("EctoSwerve/ExponentialOdometry/X").SetDouble(expoOdometry.getPose().getX());
	table->GetEntry("EctoSwerve/ExponentialOdometry/Y").SetDouble(expoOdometry.getPose().getY());
	
	table->GetEntry("EctoSwerve/Wheels/TopLeft/Angle").SetDouble(motorStates.topLeft.wheelAngle);
	table->GetEntry("EctoSwerve/Wheels/TopRight/Angle").SetDouble(motorStates.topRight.wheelAngle);
	table->GetEntry("EctoSwerve/Wheels/BackLeft/Angle").SetDouble(motorStates.backLeft.wheelAngle);
	table->GetEntry("EctoSwerve/Wheels/BackRight/Angle").SetDouble(motorStates.backRight.wheelAngle);
	
	table->GetEntry("EctoSwerve/Wheels/TopLeft/Velocity").SetDouble(motorStates.topLeft.wheelVelocity);
	table->GetEntry("EctoSwerve/Wheels/TopRight/Velocity").SetDouble(motorStates.topRight.wheelVelocity);
	table->GetEntry("EctoSwerve/Wheels/BackLeft/Velocity").SetDouble(motorStates.backLeft.wheelVelocity);
	table->GetEntry("EctoSwerve/Wheels/BackRight/Velocity").SetDouble(motorStates.backRight.wheelVelocity);
	
	table->GetEntry("EctoSwerve/Wheels/TopLeft/Current").SetDouble(frontLeftWheel->getMotorCurrent());
	table->GetEntry("EctoSwerve/Wheels/TopRight/Current").SetDouble(frontRightWheel->getMotorCurrent());
	table->GetEntry("EctoSwerve/Wheels/BackLeft/Current").SetDouble(backLeftWheel->getMotorCurrent());
	table->GetEntry("EctoSwerve/Wheels/BackRight/Current").SetDouble(backRightWheel->getMotorCurrent());
	
	auto pidError = getSteerPIDError();
	table->GetEntry("EctoSwerve/Wheels/TopLeft/AngleError").SetDouble(pidError.frontLeft);
	table->GetEntry("EctoSwerve/Wheels/TopRight/AngleError").SetDouble(pidError.frontRight);
	table->GetEntry("EctoSwerve/Wheels/BackLeft/AngleError").SetDouble(pidError.backLeft);
	table->GetEntry("EctoSwerve/Wheels/BackRight/AngleError").SetDouble(pidError.backRight);
	
	odometryPreviousTime = frc::Timer::GetFPGATimestamp();
}

void EctoSwerve::updateInitialPositionChooser() {
	odometryLock.lock();
	log->info("Position Reset!");
	odometry.resetPosition(initialPositionChooser.GetSelected());
	headingZero = -initialPositionChooser.GetSelected().getHeading().getRadians(); //Offset yaw
	expoOdometry = ExponentialOdometry(initialPositionChooser.GetSelected());
	log->info("{},{},{}", initialPositionChooser.GetSelected().getX(), initialPositionChooser.GetSelected().getY(),
	          initialPositionChooser.GetSelected().getHeading().getRadians());
	odometryLock.unlock();
}

GenericSwerveValue EctoSwerve::getWheelCurrents() const {
	GenericSwerveValue out;
	out.frontLeft = frontLeftWheel->getMotorCurrent();
	out.frontRight = frontRightWheel->getMotorCurrent();
	out.backLeft = backLeftWheel->getMotorCurrent();
	out.backRight = backRightWheel->getMotorCurrent();
	return out;
}

GenericSwerveValue EctoSwerve::getSteerCurrents() const {
	GenericSwerveValue out;
	out.frontLeft = frontLeftSteer->getMotorCurrent();
	out.frontRight = frontRightSteer->getMotorCurrent();
	out.backLeft = backLeftSteer->getMotorCurrent();
	out.backRight = backRightSteer->getMotorCurrent();
	return out;
}

GenericSwerveValue EctoSwerve::getSteerPIDError() const {
	GenericSwerveValue out;
	out.frontLeft = motorStates.topLeft.wheelAngle - lastMotorSetpoint.topLeft.wheelAngle;
	out.frontRight = motorStates.topRight.wheelAngle - lastMotorSetpoint.topRight.wheelAngle;
	out.backLeft = motorStates.backLeft.wheelAngle - lastMotorSetpoint.backLeft.wheelAngle;
	out.backRight = motorStates.backRight.wheelAngle - lastMotorSetpoint.backRight.wheelAngle;
	return out;
}

GenericSwerveValue EctoSwerve::getWheelPIDError() const {
	GenericSwerveValue out;
	out.frontLeft = motorStates.topLeft.wheelVelocity - lastMotorSetpoint.topLeft.wheelVelocity;
	out.frontRight = motorStates.topRight.wheelVelocity - lastMotorSetpoint.topRight.wheelVelocity;
	out.backLeft = motorStates.backLeft.wheelVelocity - lastMotorSetpoint.backLeft.wheelVelocity;
	out.backRight = motorStates.backRight.wheelVelocity - lastMotorSetpoint.backRight.wheelVelocity;
	return out;
}