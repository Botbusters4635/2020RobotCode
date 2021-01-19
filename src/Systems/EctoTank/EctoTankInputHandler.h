//
// Created by abiel on 7/15/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOTANKINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_ECTOTANKINPUTHANDLER_H

#include "EctoTank.h"
#include <Core/EctoModule/System.h>

#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

/**
 * Probably shouldn't be used on a real robot, as it only maps
 * 3 axes from the joy into vX, vY, vTheta (kinda like arcade mode)
 * Pretty please don't use this in a real robot.
 */
/**
 * Input bindings:
 * leftX -> vX
 * leftY -> vTheta
 */

struct EctoTankInputHandlerConfig {
	double maxLinearVelocity = 5.0;
	double maxAngularVelocity = M_PI;
};

class EctoTankInputHandler : public System {
public:
	EctoTankInputHandler(const std::shared_ptr<EctoTank> &tank, const EctoTankInputHandlerConfig &config);
	
	EctoTankInputHandler(const std::shared_ptr<EctoTank> &tank);
	
	void initRobot() override;
	
	void updateRobot() override;

private:
	EctoTankInputHandlerConfig config;
	
	InputManager &input = InputManager::getInstance();
	std::shared_ptr<EctoTank> tank;
	
	JoystickAxisExpo* vXJoy, *vThetaJoy;
	
	/*
	 * Use this to change input bindings
	 */
	static constexpr auto vXJoyName = "leftX";
	static constexpr auto vThetaJoyName = "leftY";
};


#endif //BOTBUSTERSREBIRTH_ECTOTANKINPUTHANDLER_H
