//
// Created by abiel on 1/2/20.
//

#include "EctoSwerveInputHandler.h"

EctoSwerveInputHandler::EctoSwerveInputHandler(EctoSwerve &swerveIn) : System("EctoSwerveRebirthInputHandler"),
                                                                       swerve(swerveIn), fieldOrientedEnable(true) {
	VisionAlignmentConfig visionAlignHelperConfig;
	visionAlignHelperConfig.vXPIDConfig.p = 1.5;
	visionAlignHelperConfig.vXPIDConfig.d = 0.000100;
	
	visionAlignHelperConfig.vYPIDConfig.p = 1.735;
	visionAlignHelperConfig.vXPIDConfig.d = 0.000100;
	
	visionAlignHelperConfig.thetaPIDConfig.p = 0.9;
	visionAlignHelperConfig.thetaPIDConfig.d = 0.000100;
	
	visionAlignHelper = std::make_unique<VisionAlignmentHelper>(visionAlignHelperConfig);
	
	const double joystickExpo = 0.25;
	const double joystickDeadzone = 0.08;
	
	const double triggerExpo = 0.2;
	const double triggerDeadzone = 0.2;
	
	strafeAxis = std::make_unique<JoystickAxisExpo>(joystickExpo, joystickDeadzone);
	forwardAxis = std::make_unique<JoystickAxisExpo>(joystickExpo, joystickDeadzone);
	rotationAxis = std::make_unique<JoystickAxisExpo>(joystickExpo, joystickDeadzone);
	
	brakeTrigger = std::make_unique<JoystickAxisExpo>(triggerExpo, triggerDeadzone);
	
	input.registerAxis(strafeAxis.get(), "rightY");
	input.registerAxis(forwardAxis.get(), "rightX");
	input.registerAxis(rotationAxis.get(), "leftY");
	
	input.registerAxis(brakeTrigger.get(), "leftTrig");
	
	input.registerButton(&fastMode, "leftPush");
	//input.registerButton(&fieldOrientedEnable, "rightJoy");
	input.registerButton(&visionEnable, "rightPush");
	input.registerButton(&resetYaw, "select");
	
	thetaPID.EnableContinuousInput(-M_PI, M_PI);
}

void EctoSwerveInputHandler::initRobot() {
	table = ntInstance.GetTable("EctoSwerveInputHandler");
//	table->GetEntry("HeadingTarget").SetDouble(0);
//	table->GetEntry("HeadingP").SetDouble(0);
//	table->GetEntry("HeadingI").SetDouble(0);
//	table->GetEntry("HeadingD").SetDouble(0);
	
}

void EctoSwerveInputHandler::updateRobot() {

//	thetaPID.SetP(table->GetEntry("HeadingP").GetDouble(0));
//	thetaPID.SetI(table->GetEntry("HeadingI").GetDouble(0));
//	thetaPID.SetD(table->GetEntry("HeadingD").GetDouble(0));
	
	const double dt = frc::Timer::GetFPGATimestamp() - lastTime;
	
	Twist2D joystickInput(forwardAxis->get(), strafeAxis->get(), -rotationAxis->get());
	
	table->GetEntry("RotationAxis").SetDouble(-rotationAxis->get());
	
	
	if (lastVisionEnable and !visionEnable.get()) {
		//Previously enabled
		visionAlignHelper->disable();
	}
	
	if (visionEnable.get()) {
		headingTarget = swerve.getYaw();
		if (!lastVisionEnable) {
			//Previously not enabled
			visionAlignHelper->reset();
			visionAlignHelper->enable();
			
			currentSetpoint.desiredDistance = visionAlignHelper->getCurrentDistance();
		}
		
		//Hardcoded for now (testing)
		currentSetpoint.desiredDistance += joystickInput.getDx() * maxVisionDistanceIncreaseRate * dt;
		table->GetEntry("CurrentSetpoint").SetDouble(currentSetpoint.desiredDistance);
		
		visionAlignHelper->setChassisVelocity(swerve.getFieldOrientedVelocity());
		visionAlignHelper->changeSetpoint(currentSetpoint);
		
		Twist2D output;
		output = visionAlignHelper->getCurrentOutput();
		
		const double maxVel = std::max(std::abs(joystickInput.getDx()), std::abs(joystickInput.getDy()));
		const double maximumAngularVelocity = std::abs(joystickInput.getDtheta());
		
		if (maxVel > maximumVisionLinearVelocity) {
			const double scaleFactor = maximumVisionLinearVelocity / maxVel;
			
			output.setDx(joystickInput.getDx() * scaleFactor);
			output.setDy(joystickInput.getDy() * scaleFactor);
		}
		
		if (maximumAngularVelocity > maximumVisionAngularVelocity) {
			output.setDtheta(std::copysign(maximumVisionAngularVelocity, output.getDtheta()));
		}
		
		output.setDy(joystickInput.getDy() * 1.8);
		output.setDx(joystickInput.getDx() * 2.6);
		
		swerve.setTargetVelocity(output, Point2D(0, 0), false);
	} else {
		Twist2D output = joystickInput;
		headingTarget += -rotationAxis->get() * M_PI / 2 * dt;
		
		if (headingTarget > M_PI) {
			headingTarget -= M_PI * 2;
		} else if (headingTarget < -M_PI) {
			headingTarget += M_PI * 2;
		}
		
		
		table->GetEntry("HeadingTarget").SetDouble(headingTarget);
		
		thetaPID.SetSetpoint(headingTarget);
		//output *= 4.4;
		output.setDtheta(output.getDtheta() * 0.75);
		//output.setDtheta(thetaPID.Calculate(swerve.getYaw()));
		
		
		if (!fastMode.get()) {
			output *= slowModeReduction;
		}
		
		if (std::abs(brakeTrigger->get()) > 0) {
			const double reductionFactor = std::max((1.0 - brakeTrigger->get()), minimumBrakeValue);
			output *= reductionFactor;
		}
		
		if (resetYaw.get()) {
			swerve.zeroYaw();
			headingTarget = 0.0;
		}
		
		
		if (filterInputs) {
			output.setDx(xFilter.Calculate(units::meters_per_second_t(output.getDx())).to<double>());
			output.setDy(yFilter.Calculate(units::meters_per_second_t(output.getDy())).to<double>());
			output.setDtheta(thetaFilter.Calculate(units::radians_per_second_t(output.getDtheta())).to<double>());
			//output.setDtheta(thetaPID.Calculate(swerve.getYaw()));
		}
		
		//output = Twist2D(1, 0, 0);
		
		swerve.setTargetVoltage(output, Point2D(0, 0), true);
	}
	
	lastVisionEnable = visionEnable.get();
	
	lastTime = frc::Timer::GetFPGATimestamp();
	
	
	//log->info("{}, {}, {}", output.getDx(), output.getDy(), output.getDtheta());
}