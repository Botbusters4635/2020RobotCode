//
// Created by hiram on 9/08/19.
//

#include <stdexcept>
#include "EctoMotorMode.h"

std::string getStringFromControlMode(MotorControlMode mode) {
	switch (mode) {
		case MotorControlMode::Percent:
			return "Percent";
		case MotorControlMode::Velocity:
			return "Velocity";
		case MotorControlMode::Position:
			return "Position";
		default:
			throw std::invalid_argument("Control mode not yet implemented");
	}
}

MotorControlMode getControlModeFromString(const std::string &mode) {
	if (mode == "Percent") return MotorControlMode::Percent;
	else if (mode == "Velocity") return MotorControlMode::Velocity;
	else if (mode == "Position") return MotorControlMode::Position;
	else throw std::invalid_argument(mode + " is not a valid control mode, options are Percent, Velocity and Position");
}
