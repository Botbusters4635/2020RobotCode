//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H

#include "EctoSwerve.h"
#include "Core/EctoModule/System.h"
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Buttons/ToggleButton.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "Utilities/VisionAlignment/VisionAlignmentHelper.h"

#include <frc/SlewRateLimiter.h>
#include <frc/PIDController.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

class EctoSwerveInputHandler : public System {
public:
	EctoSwerveInputHandler(EctoSwerve &swerveIn);
	
	void initRobot() override;
	
	void updateRobot() override;

private:
	EctoSwerve &swerve;
	
	InputManager &input = InputManager::getInstance();
	
	/**
	 * NT
	 */
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	std::unique_ptr<JoystickAxisExpo> strafeAxis, forwardAxis, rotationAxis;
	std::unique_ptr<JoystickAxisExpo> brakeTrigger;
	
	const double slowModeReduction = 0.9;
	const double minimumBrakeValue = 0.15;
	
	ToggleButton fieldOrientedEnable;
	EctoButton visionEnable, resetYaw, fastMode;
	
	const bool filterInputs = true;
	units::meters_per_second_t maximumInputAcceleration{30.0}; // / 1s
	units::radians_per_second_t maximumInputAngularAcceleration{M_PI * 8}; // / 1S
	
	frc::SlewRateLimiter<units::meters_per_second> xFilter{maximumInputAcceleration / units::second_t(1.0)};
	frc::SlewRateLimiter<units::meters_per_second> yFilter{maximumInputAcceleration / units::second_t(1.0)};
	frc::SlewRateLimiter<units::radians_per_second> thetaFilter{maximumInputAngularAcceleration / units::second_t(1.0)};
	double headingTarget = 0.0;
	frc2::PIDController thetaPID{3.5, 0.0, 0.015};
	std::unique_ptr<VisionAlignmentHelper> visionAlignHelper;
	bool lastVisionEnable = false;
	
	const double maximumVisionLinearVelocity = 2.75;
	const double maximumVisionAngularVelocity = M_PI * 1.5;
	
	VisionAlignmentSetpoint currentSetpoint;
	
	const double maxVisionDistanceIncreaseRate = 1.67;
	
	double lastTime;
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
