//
// Created by abiel on 2/17/20.
//

#include "VisionAlignmentHelper.h"

VisionAlignmentHelper::VisionAlignmentHelper(const VisionAlignmentConfig &config) {
	this->config = config;
	
	thetaPID = std::make_unique<frc2::PIDController>(config.thetaPIDConfig.p, config.thetaPIDConfig.i,
	                                                 config.thetaPIDConfig.d, kDt);
	vXPID = std::make_unique<frc2::PIDController>(config.vXPIDConfig.p, config.vXPIDConfig.i, config.vXPIDConfig.d,
	                                              kDt);
	vYPID = std::make_unique<frc2::PIDController>(config.vYPIDConfig.p, config.vYPIDConfig.i, config.vYPIDConfig.d,
	                                              kDt);
	
	vXPID->SetTolerance(0.05);
	vYPID->SetTolerance(0.1);
	thetaPID->SetTolerance(0.01);
	
	thetaPID->EnableContinuousInput(-M_PI, M_PI);
	
	table = ntInstance.GetTable("VisionAlignmentHelper");
	table->GetEntry("thetaPID/P").SetDouble(config.thetaPIDConfig.p);
	table->GetEntry("thetaPID/I").SetDouble(config.thetaPIDConfig.i);
	table->GetEntry("thetaPID/D").SetDouble(config.thetaPIDConfig.d);
	table->GetEntry("vXPID/P").SetDouble(config.vXPIDConfig.p);
	table->GetEntry("vXPID/I").SetDouble(config.vXPIDConfig.i);
	table->GetEntry("vXPID/D").SetDouble(config.vXPIDConfig.d);
	table->GetEntry("vYPID/P").SetDouble(config.vYPIDConfig.p);
	table->GetEntry("vYPID/I").SetDouble(config.vYPIDConfig.i);
	table->GetEntry("vYPID/D").SetDouble(config.vYPIDConfig.d);
	
	updateNotifier = std::make_unique<frc::Notifier>(std::bind(&VisionAlignmentHelper::update, this));
}

void VisionAlignmentHelper::enable() {
	if (!isEnabled) {
		updateNotifier->StartPeriodic(kDt);
		isEnabled = true;
	}
	
	visionManager.setCameraLEDRingState(true);
}

void VisionAlignmentHelper::disable() {
	if (isEnabled) {
		updateNotifier->Stop();
		isEnabled = false;
	}
	
	visionManager.setCameraLEDRingState(false);
}

void VisionAlignmentHelper::reset() {
	thetaPID->Reset();
	vYPID->Reset();
	vXPID->Reset();
}

void VisionAlignmentHelper::changeSetpoint(const VisionAlignmentSetpoint &newSetpoint) {
	vXPID->SetSetpoint(newSetpoint.desiredDistance);
	vYPID->SetSetpoint(0);
	thetaPID->SetSetpoint(newSetpoint.desiredCircleAngle);
}

void VisionAlignmentHelper::setChassisVelocity(const Twist2D &currentVelocity) {
	this->currentVelocity = currentVelocity;
}

Twist2D VisionAlignmentHelper::calculateOutput() {
	visionManager.setCameraLEDRingState(true);
	
	const VisionDataContainer latestMessage = visionManager.getLatestData();
	RobotPose2D compensatedPose = VisionManager::correctDataForLatency(latestMessage, currentVelocity);
	
	const double robotTheta = thetaPID->Calculate(latestMessage.robotPose.getHeading().getRadians());
	
	toTargetMutex.lock();
	distanceToTarget = latestMessage.robotPose.getX();
	angleToTarget = latestMessage.robotPose.getY();
	toTargetMutex.unlock();
	
	
	Twist2D output(0, 0, 0);
	
	table->GetEntry("RotToTarget").SetDouble(latestMessage.robotPose.getHeading().getRadians());
	table->GetEntry("DistanceToTarget").SetDouble(distanceToTarget);
	table->GetEntry("Y").SetDouble(compensatedPose.getY());
	
	thetaPID->SetP(table->GetEntry("thetaPID/P").GetDouble(config.vYPIDConfig.p));
	thetaPID->SetI(table->GetEntry("thetaPID/I").GetDouble(config.thetaPIDConfig.i));
	thetaPID->SetD(table->GetEntry("thetaPID/D").GetDouble(config.thetaPIDConfig.d));
	vXPID->SetP(table->GetEntry("vXPID/P").GetDouble(config.vXPIDConfig.p));
	vXPID->SetI(table->GetEntry("vXPID/I").GetDouble(config.vXPIDConfig.i));
	vXPID->SetD(table->GetEntry("vXPID/D").GetDouble(config.vXPIDConfig.d));
	vYPID->SetP(table->GetEntry("vYPID/P").GetDouble(config.vYPIDConfig.p));
	vYPID->SetI(table->GetEntry("vYPID/I").GetDouble(config.vYPIDConfig.i));
	vYPID->SetD(table->GetEntry("vYPID/D").GetDouble(config.vYPIDConfig.d));
	
	targetDetected.store(latestMessage.isDetected);
	
	if (latestMessage.isDetected) {
		
		output.setDx(-vXPID->Calculate(distanceToTarget));
		output.setDy(-vYPID->Calculate(robotTheta));
		output.setDtheta(-thetaPID->Calculate(latestMessage.robotPose.getY()));
		
		if (vXPID->AtSetpoint()) {
			output.setDx(0);
		}
		
		if (vYPID->AtSetpoint()) {
			output.setDy(0);
		}
		
		
		if (thetaPID->AtSetpoint()) {
			output.setDtheta(0);
		}
		
		Twist2D outputRotated = output;
		
		outputRotated.setDx(output.getDx() * std::cos(robotTheta) - output.getDy() * std::sin(robotTheta));
		outputRotated.setDy(output.getDx() * std::sin(robotTheta) + output.getDy() * std::cos(robotTheta));
		
		output = outputRotated;
	}
	
	if (isTargetDetected() and std::abs(distanceToTarget - 3.15) < 0.25) {
		PatternCommand command;
		command.primaryPattern = LEDPattern::BreatheRandom;
		command.primaryColor = RGBColor(0, 0, 255);
		manager.queueCommand(command, PatternPriority::HighPriority);
	} else {
		manager.stopCommandsByPriority(PatternPriority::HighPriority);
	}
	
	return output;
}

bool VisionAlignmentHelper::isThetaAtSetpoint() const {
	return thetaPID->AtSetpoint();
}

double VisionAlignmentHelper::getCurrentDistance() const {
	std::lock_guard<std::mutex> lockGuard(toTargetMutex);
	return distanceToTarget;
}

double VisionAlignmentHelper::getAngleToTarget() const {
	std::lock_guard<std::mutex> lockGuard(toTargetMutex);
	return angleToTarget;
}

double VisionAlignmentHelper::getXError() const {
	return vXPID->GetPositionError();
}

double VisionAlignmentHelper::getYError() const {
	return vYPID->GetPositionError();
}

double VisionAlignmentHelper::getThetaError() const {
	return thetaPID->GetPositionError();
}

bool VisionAlignmentHelper::isTargetDetected() const {
	return targetDetected.load();
}

void VisionAlignmentHelper::update() {
	outputLock.lock();
	currentOutput = calculateOutput();
	outputLock.unlock();
}

Twist2D VisionAlignmentHelper::getCurrentOutput() const {
	std::lock_guard<std::mutex> lockGuard(outputLock);
	return currentOutput;
}