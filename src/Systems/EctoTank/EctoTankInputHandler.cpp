//
// Created by abiel on 7/15/20.
//

#include "EctoTankInputHandler.h"

EctoTankInputHandler::EctoTankInputHandler(const std::shared_ptr<EctoTank> &tank,
                                           const EctoTankInputHandlerConfig &config)
		: System("EctoTankInputHandler", false) {
	if (tank == nullptr)
		throw std::runtime_error("EctoTankInputHandler given nullptr as EctoTank");
	
	this->tank = tank;
	this->config = config;
}

EctoTankInputHandler::EctoTankInputHandler(const std::shared_ptr<EctoTank> &tank) : EctoTankInputHandler(tank, {}) {
	;
}

void EctoTankInputHandler::initRobot() {
	vXJoy = new JoystickAxisExpo(0.1, 0.1);
	vThetaJoy = new JoystickAxisExpo(0.1, 0.1);
	
	input.registerAxis(vXJoy, vXJoyName);
	input.registerAxis(vThetaJoy, vThetaJoyName);
}

void EctoTankInputHandler::updateRobot() {
	Twist2D vel;
	
	vel.setDx(-vXJoy->get() * config.maxLinearVelocity);
	vel.setDtheta(-vThetaJoy->get() * config.maxAngularVelocity);
	
	tank->setTargetVelocity(vel);
}