//
// Created by abiel on 2/10/20.
//

#ifndef BOTBUSTERSREBIRTH_BALLINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_BALLINPUTHANDLER_H

#include "Systems/BallFeeder/BallFeeder.h"
#include "Systems/BallIntake/BallIntake.h"
#include "Systems/PIDShooter/PidShooter.h"
#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoModule/System.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

struct BallInputHandlerConfig {
	std::shared_ptr<BallFeeder> feeder;
	std::shared_ptr<BallIntake> intake;
	std::shared_ptr<PIDShooter> shooter;
	
	double maxVelOnSwitchIntake = 0.5;
};

class BallInputHandler : public System {
public:
	BallInputHandler(const BallInputHandlerConfig &config);
	
	void initRobot() override;
	
	void updateRobot() override;

private:
	InputManager &input = InputManager::getInstance();
	
	JoystickAxisExpo ballFeeder{0.25, 0.05};
	JoystickAxisExpo intakeSpeedControl{0.25, 0.05};
	
	BallInputHandlerConfig config;
	
	EctoButton leftIntakeEnable, rightIntakeEnable;
	
	EctoButton stopBallsOnSwitch, lessShoot, closeShoot, longShoot;
	EctoButton stopFlywheel;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table, swerveTable, elevatorTable;
	
	bool facingForward = true;
};


#endif //BOTBUSTERSREBIRTH_BALLINPUTHANDLER_H
