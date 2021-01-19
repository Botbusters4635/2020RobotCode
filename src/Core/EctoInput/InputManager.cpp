#include <iostream>
#include "InputManager.h"
//TODO Re-implement input checking???

InputManager::InputManager() : Manager("InputManager") {
	log->info("Initializing EctoInput...");
	
	joystickCount = settings->getNumber("Input", "StickCount");
	
	initializeJoysticks();
	initializeAxis();
	initializeButtons();
}

void InputManager::update() {
	if (driverStation.IsNewControlData()) {
		for (const auto &joystick : joysticksData) {
			const JoystickData &joyData = joystick.second;
			
			for (const auto &buttonData : joyData.buttonsToUpdate) {
				
				const bool buttonState = isButtonPressed(buttonData.first, joystick.first);
				
				for (auto &button : buttonData.second) {
					button->updateStatus(buttonState);
				}
			}
			
			for (const auto &axisData : joyData.axesToUpdate) {
				
				const double axisValue = readAxisValue((unsigned int) axisData.first, joystick.first);
				
				for (const auto &axis : axisData.second) {
					
					axis->updateValue(axisValue);
					
				}
			}
		}
	}
}

void InputManager::registerButton(EctoButton *button, const std::string &buttonName, int joystickId) {
	std::lock_guard<std::mutex> lock(ectoInputLock);
	
	if (joysticksData.count(joystickId) == 0) {
		throw std::runtime_error("Joystick ID " + std::to_string(joystickId) + " not found in EctoInput config");
	}
	JoystickData &data = joysticksData.at(joystickId);
	
	if (data.buttonNames.count(buttonName) == 0) {
		throw std::runtime_error(
				"Joystick ID " + std::to_string(joystickId) + " does not have a button named " + buttonName +
				" in EctoInput config");
	}
	
	int buttonId = data.buttonNames.at(buttonName);
	
	data.buttonsToUpdate[buttonId].emplace_back(button);
}

void InputManager::registerAxis(JoystickAxis *axis, const std::string &axisName, int joystickId) {
	std::lock_guard<std::mutex> lock(ectoInputLock);
	
	if (joysticksData.count(joystickId) == 0) {
		throw std::runtime_error("Joystick ID " + std::to_string(joystickId) + " not found in EctoInput config");
	}
	JoystickData &data = joysticksData.at(joystickId);
	
	if (data.axesNames.count(axisName) == 0) {
		throw std::runtime_error(
				"Joystick ID " + std::to_string(joystickId) + " does not have a axis named" + axisName +
				"inEctoInput config");
	}
	
	int axisId = data.axesNames.at(axisName);
	
	data.axesToUpdate[axisId].emplace_back(axis);
}

void InputManager::setControllerRumble(double leftRumble, double rightRumble, int joystickId) {
	joysticksData.at(joystickId).joystick->SetRumble(frc::GenericHID::kLeftRumble, leftRumble);
	joysticksData.at(joystickId).joystick->SetRumble(frc::GenericHID::kRightRumble, rightRumble);
}

void InputManager::initializeJoysticks() {
	std::lock_guard<std::mutex> lock(ectoInputLock);
	
	log->info("Initializing Sticks...");
	for (int i = 1; i <= joystickCount; ++i) {
		auto stickId = settings->getNumber("Input", baseNameStick + std::to_string(i));
		
		JoystickData joystickData;
		
		joystickData.joystickId = i;
		joystickData.wpiJoystickId = (int) stickId;
		
		joystickData.joystick = std::make_unique<frc::Joystick>((int) stickId);

#ifndef SIMULATION
		//PC
		
		while (joystickData.joystick->GetAxisCount() == 0) {
			log->error("Waiting for joystick {}!", stickId);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		
		joystickData.axisCount = joystickData.joystick->GetAxisCount();
		joystickData.buttonCount = joystickData.joystick->GetButtonCount();

#else //Roborio
		joystickData.axisCount = 6;
		joystickData.buttonCount = 10;
#endif
		
		joysticksData.emplace(i, joystickData);
	}
}

void InputManager::initializeAxis() {
	std::lock_guard<std::mutex> lock(ectoInputLock);
	
	for (std::pair<const int, JoystickData> &data : joysticksData) {
		JoystickData &joyData = data.second;
		
		for (int i = 1; i <= joyData.axisCount; ++i) {
			std::string axisName = settings->getString("Input",
			                                           baseNameStickSelector + std::to_string(data.first) + "." +
			                                           baseNameAxis + std::to_string(i));
			
			joyData.axesNames.emplace(std::make_pair(axisName, i));
		}
	}
}

void InputManager::initializeButtons() {
	std::lock_guard<std::mutex> lock(ectoInputLock);
	
	for (std::pair<const int, JoystickData> &data : joysticksData) {
		JoystickData &joyData = data.second;
		
		for (int i = 1; i <= joyData.buttonCount; ++i) {
			std::string buttonName = settings->getString("Input",
			                                             baseNameStickSelector + std::to_string(data.first) + "." +
			                                             baseNameButton + std::to_string(i));
			
			joyData.buttonNames.emplace(std::make_pair(buttonName, i));
		}
	}
}

bool InputManager::isButtonPressed(int buttonID, int joystickID) const {
	return joysticksData.at(joystickID).joystick->GetRawButton(buttonID);
}

double InputManager::readAxisValue(int axisID, int joystickID) const {
	return joysticksData.at(joystickID).joystick->GetRawAxis(axisID - 1);
}

int InputManager::getPOVAngleReading(int joystickId) const {
	return joysticksData.at(joystickId).joystick->GetPOV();
}
