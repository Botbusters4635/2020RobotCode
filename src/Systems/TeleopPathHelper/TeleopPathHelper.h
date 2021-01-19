//
// Created by abiel on 2/22/20.
//

#ifndef BOTBUSTERSREBIRTH_TELEOPPATHHELPER_H
#define BOTBUSTERSREBIRTH_TELEOPPATHHELPER_H

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/EctoModule/System.h"

#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Buttons/EctoButton.h>


class TeleopPathHelper : public System {
public:
	TeleopPathHelper(const std::shared_ptr<EctoSwerve> &swerve);
	
	void updateRobot();

private:
	std::shared_ptr<EctoSwerve> swerve;
	
	InputManager &input = InputManager::getInstance();
	
	EctoButton logButton;
	bool buttonLastPressed{false};
};


#endif //BOTBUSTERSREBIRTH_TELEOPPATHHELPER_H
