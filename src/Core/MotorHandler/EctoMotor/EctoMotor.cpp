//
// Created by hiram on 9/08/19.
//

#include <stdexcept>
#include <cmath>
#include <chrono>
#include "EctoMotor.h"

std::shared_ptr<spdlog::logger> EctoMotor::log = spdlog::stdout_color_mt("EctoMotor");

EctoMotor::EctoMotor(int motorID, const std::string &motorName, EctoMotorType motorType) {
	this->id = motorID;
	this->name = motorName;
	this->motorType = motorType;
	
	log->trace("Creating motor with name {} and id {}...", motorName, motorID);
}

void EctoMotor::set(double value) {
	this->set(value, this->controlMode);
}

void EctoMotor::set(double value, MotorControlMode newControlMode) {
	this->controlMode = newControlMode;
	
	lastSetpoint = std::make_pair(newControlMode, value);
	
	switch (this->controlMode) {
		case MotorControlMode::Percent:
			this->setPercentOutput(value);
			break;
		case MotorControlMode::Velocity:
			this->setVelocityOutput(value);
			break;
		case MotorControlMode::Position:
			this->setPositionOutput(value);
			break;
		case MotorControlMode::MotionMagic:
			this->setMotionMagicOutput(value);
			break;
		case MotorControlMode::Current:
			this->setMotorCurrentOutput(value);
			break;
		case MotorControlMode::Voltage:
			this->setVoltageOutput(value);
			break;
		default:
			throw std::logic_error("Invalid MotorControlMode");
	}
}

std::string EctoMotor::getName() const {
	return name;
}

void EctoMotor::setControlMode(MotorControlMode controlMode) {
	this->controlMode = controlMode;
}

MotorControlMode EctoMotor::getControlMode() const {
	return this->controlMode;
}

void EctoMotor::setDefaultFeedbackMode(MotorFeedbackMode feedbackMode) {
	this->feedbackMode = feedbackMode;
	switch (this->feedbackMode) {
		case MotorFeedbackMode::None:
			throw std::runtime_error("Cant set feedbackmode to None in motor with name: " + name);
		case MotorFeedbackMode::QuadEncoder:
			this->setQuadAsClosedLoopSource();
			break;
		case MotorFeedbackMode::Potentiometer:
			this->setPotAsClosedLoopSource();
			break;
	}
}

MotorFeedbackMode EctoMotor::getDefaultFeedbackMode() const {
	return this->feedbackMode;
}

int EctoMotor::getId() const {
	return id;
}

bool EctoMotor::isMotorDisabled() const {
	return disabled;
}

double EctoMotor::getPosition() const {
//	if (getEncoderCodes() == 0) {
//		log->warn("Trying to get position but no encoder codes defined! on motor: {}", name);
//		return 0;
//	}
	
	switch (feedbackMode) {
		case MotorFeedbackMode::None:
			log->warn("Cannot get a position when no feedback mode is defined in motor with name: {} ", name);
			return 0;
		case MotorFeedbackMode::QuadEncoder:
			return getQuadPosition();
		case MotorFeedbackMode::Potentiometer:
			return getPotPosition();
		default:
			return 0;
	}
}

double EctoMotor::getVelocity() const {
	//if (getEncoderCodes() == 0) {
	//	log->warn("Trying to get position but no encoder codes defined! on motor: {}", name);
	//	return 0;
	//}
	
	switch (feedbackMode) {
		case MotorFeedbackMode::None:
			log->warn("Cannot get a velocity when no feedback mode is defined in motor with name: {} ", name);
			return 0;
		case MotorFeedbackMode::QuadEncoder:
			return getQuadVelocity();
		case MotorFeedbackMode::Potentiometer:
			return getPotVelocity();
		default:
			return 0;
	}
}

void EctoMotor::setControllerSourceMode(MotorControlMode mode) {
	this->controllerSourceMode = mode;
}

void EctoMotor::PIDWrite(double output) {
	this->set(output, MotorControlMode::Percent);
}

void EctoMotor::outputSet(double setpoint) {
	this->PIDWrite(setpoint);
}

double EctoMotor::PIDGet() {
	return getPosition();
}

EctoMotorType EctoMotor::getMotorType() const {
	return motorType;
}

std::pair<MotorControlMode, double> EctoMotor::getLastSetpoint() const {
	return lastSetpoint;
}
