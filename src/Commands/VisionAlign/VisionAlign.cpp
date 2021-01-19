//
// Created by abiel on 2/18/20.
//

#include "VisionAlign.h"

VisionAlign::VisionAlign(const shared_ptr<EctoSwerve> &swerve, const VisionAlignmentConfig &alignHelperConfig) {
	this->swerve = swerve;
	visionAlignHelper = std::make_shared<VisionAlignmentHelper>(alignHelperConfig);
	
	setpoint.desiredCircleAngle = 0.0;
	setpoint.desiredDistance = visionAlignHelper->getCurrentDistance();
	
	table = ntInstance.GetTable("VisionAlign");
	
	visionAlignHelper->enable();
}

VisionAlign::VisionAlign(const std::shared_ptr<EctoSwerve> &swerve, const VisionAlignmentConfig &alignHelperConfig,
                         double targetDistance, double targetCircleAngle)
		: VisionAlign(swerve, alignHelperConfig) {
	setpoint.desiredCircleAngle = targetCircleAngle;
	setpoint.desiredDistance = targetDistance;
}

VisionAlign::VisionAlign(const std::shared_ptr<EctoSwerve> &swerve, const VisionAlignmentConfig &alignHelperConfig,
                         double targetDistance) : VisionAlign(swerve, alignHelperConfig) {
	setpoint.desiredDistance = targetDistance;
}

void VisionAlign::Initialize() {
	startTime = frc::Timer::GetFPGATimestamp();
	visionAlignHelper->changeSetpoint(setpoint);
}

void VisionAlign::Execute() {
	Twist2D output = visionAlignHelper->getCurrentOutput();
	output.setDy(0);
	output.setDx(0);
	
	swerve->setTargetVelocity(output, Point2D(0, 0), false);
}

void VisionAlign::End(bool interrupted) {
	visionAlignHelper->disable();
	swerve->setTargetVelocity({0, 0, 0});
}

bool VisionAlign::IsFinished() {
	if (frc::Timer::GetFPGATimestamp() - startTime > 0.5) {
		return true;
	}
	
	const double currentLinearError = std::abs(
			this->setpoint.desiredDistance - visionAlignHelper->getCurrentDistance());
	const double currentAngularError = std::abs(visionAlignHelper->getThetaError());
	
	table->GetEntry("CurrentAngularError").SetDouble(currentAngularError);
	std::cout << currentAngularError << std::endl;
	std::cout << visionAlignHelper->isThetaAtSetpoint() << std::endl;
	std::cout << visionAlignHelper->getAngleToTarget() << std::endl;
	
	return currentAngularError < 0.1 and visionAlignHelper->isTargetDetected();
}