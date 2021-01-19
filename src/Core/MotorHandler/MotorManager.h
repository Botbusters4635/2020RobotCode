//
// Created by alberto on 31/07/19.
//
#ifndef ECTOCONTROL_MOTORHANDLER_H
#define ECTOCONTROL_MOTORHANDLER_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "EctoMotor/DataTypes/EctoMotorType.h"
#include <Core/EctoModule/Manager.h>

class MotorManager : public Manager<MotorManager> {
	friend class Manager<MotorManager>;

public:
	std::shared_ptr<EctoMotor> &getMotor(const std::string &name);
	
	void update() override;

private:
	const std::string talonVersion = "20.1";
	const std::string sparkMaxVersion = "v1.5.2";
	
	void initializeMotors();
	
	void initializeSimulatedMotors();
	
	std::map<std::string, std::string> rawMotorConfig;
	std::vector<std::shared_ptr<EctoMotor>> motorControllers;
	
	MotorManager();
};

#endif //ECTOCONTROL_MOTORHANDLER_H
