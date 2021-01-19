//
// Created by abiel on 12/30/19.
//

#include "EctoMotorSimulated.h"

EctoMotorSimulated::EctoMotorSimulated(int id, const std::string &name, const std::string &baseTableName) : EctoMotor(id, name,
                                                                                    EctoMotorType::Simulated) {
	baseTable = ntInstance.GetTable(fmt::format("{}/{}", baseTableName, name));
	
	pidTable = baseTable->GetSubTable("pid");
	
	positionEntry = baseTable->GetEntry("position");
	velocityEntry = baseTable->GetEntry("velocity");
	setpointEntry = baseTable->GetEntry("set");
	motorModeEntry = baseTable->GetEntry("motorMode");
	
	/**
	 * Write default values to nt
	 */
	PIDConfig pidConfig;
	pidTable->GetEntry("p").SetDouble(pidConfig.p);
	pidTable->GetEntry("i").SetDouble(pidConfig.i);
	pidTable->GetEntry("d").SetDouble(pidConfig.d);
	pidTable->GetEntry("f").SetDouble(pidConfig.f);
	
	pidTable->GetEntry("maxInput").SetDouble(pidConfig.maxInput);
	pidTable->GetEntry("minInput").SetDouble(pidConfig.minInput);
	pidTable->GetEntry("maxOutput").SetDouble(pidConfig.maxOutput);
}

void EctoMotorSimulated::factoryReset() {
	log->info("Factory reseted simulated motor with name: {}", name);
}

void EctoMotorSimulated::setVoltageOutput(double voltage) {
	;
}

void EctoMotorSimulated::setLimitSwitchPolarity(bool normallyClosed) {
	this->switchPolarity = normallyClosed;
}

std::string EctoMotorSimulated::getFirmwareVersion() const {
	return "Simulated";
}

void EctoMotorSimulated::invert(bool state) {
	this->inverted = state;
}

bool EctoMotorSimulated::isInverted() const {
	return inverted;
}

void EctoMotorSimulated::invertSensor(bool state) {
	this->sensorInverted = state;
}

bool EctoMotorSimulated::isSensorInverted() const {
	return sensorInverted;
}

void EctoMotorSimulated::setPIDConfig(const PIDConfig &pidConfig, int profileSlot) {
	//TODO Multiple profile slots are not supported
	if(profileSlot != 0) throw std::runtime_error("Multiple profile slots are not supported for a simulated EctoMotor");
	
	pidTable->GetEntry("p").SetDouble(pidConfig.p);
	pidTable->GetEntry("i").SetDouble(pidConfig.i);
	pidTable->GetEntry("d").SetDouble(pidConfig.d);
	pidTable->GetEntry("f").SetDouble(pidConfig.f);
	
	pidTable->GetEntry("maxInput").SetDouble(pidConfig.maxInput);
	pidTable->GetEntry("minInput").SetDouble(pidConfig.minInput);
	pidTable->GetEntry("maxOutput").SetDouble(pidConfig.maxOutput);
}

void EctoMotorSimulated::enableBrakingOnIdle(bool state) {
	this->isBrakeEnabled = state;
}

void EctoMotorSimulated::enableCurrentLimit(bool state) {
	this->isMotorCurrentLimitEnabled = state;
}

void EctoMotorSimulated::setMotorCurrentLimit(double current) {
	this->currentLimit = current;
}

void EctoMotorSimulated::setMotorCurrentOutput(double value) {
	;
}

void EctoMotorSimulated::setClosedLoopOutputRange(double minimum, double maximum) {
	this->minimumOutputRate = minimum;
	this->maximumOutputRate = maximum;
}

void EctoMotorSimulated::setClosedLoopRampRate(double rampRate) {
	this->closedLoopRampRate = rampRate;
}

void EctoMotorSimulated::setOpenLoopRampRate(double rampRate) {
	this->openLoopRampRate = rampRate;
}

void EctoMotor::setPercentOutput(double value) {
	;
}

double EctoMotorSimulated::getPercentOutput() const {
	return 0;
}

void EctoMotorSimulated::setSensorPosition(double position) {
	this->currentPosition = position;
}

void EctoMotorSimulated::setPositionOutput(double position) {
	this->positionTarget = position;
	setpointEntry.SetDouble(position);
	motorModeEntry.SetString(getStringFromControlMode(MotorControlMode::Position));
}

void EctoMotorSimulated::setVelocityOutput(double velocity) {
	this->velocityTarget = velocity;
	setpointEntry.SetDouble(velocity);
	motorModeEntry.SetString(getStringFromControlMode(MotorControlMode::Velocity));
}

double EctoMotorSimulated::getMotorTemperature() const {
	return 0;
}

double EctoMotorSimulated::getMotorCurrent() const {
	return 0;
}

double EctoMotorSimulated::getMotorVoltage() const {
	return 12.0;
}

void EctoMotorSimulated::setEncoderCodes(int codes) {
	this->encoderCodes = codes;
}

int EctoMotorSimulated::getEncoderCodes() const {
	return this->encoderCodes;
}

void EctoMotorSimulated::setArbitraryFeedForward(double feedForward) {
	;
}

void EctoMotorSimulated::disableMotor() {
	;
}

bool EctoMotorSimulated::isMotorDisabled() const {
	return false;
}

void EctoMotorSimulated::enableLimitSwitches(bool state) {
	;
}

void EctoMotorSimulated::setForwardSoftLimit(double radians) {
	;
}

void EctoMotorSimulated::setReverseSoftLimit(double radians) {
	;
}

void EctoMotorSimulated::enableReverseSoftLimit(bool state) {
	;
}

bool EctoMotorSimulated::getReverseLimitSwitch() const {
	return false;
}

bool EctoMotorSimulated::getForwardLimitSwitch() const {
	return false;
}

void EctoMotorSimulated::configureMotionMagicVelocity(double velocity) {
	;
}

void EctoMotorSimulated::configureMotionMagicAcceleration(double acceleration) {
	;
}

void EctoMotorSimulated::setMotionMagicOutput(double value) {
	;
}

double EctoMotorSimulated::getPotPosition() const {
	return 0;
}

double EctoMotorSimulated::getPotVelocity() const {
	return 0;
}

double EctoMotorSimulated::getQuadPosition() const {
	return positionEntry.GetDouble(0);
}

double EctoMotorSimulated::getQuadVelocity() const {
	return velocityEntry.GetDouble(0);
}

void EctoMotorSimulated::setPotAsClosedLoopSource() {
	;
}

void EctoMotorSimulated::setQuadAsClosedLoopSource() {
	;
}

void EctoMotorSimulated::setAnalogSensorOffset(double analogOffset) {
	;
}

void EctoMotorSimulated::enableForwardSoftLimit(bool state) {
	;
}

void EctoMotorSimulated::setAnalogPositionConversionFactor(double factor) {
	;
}

void EctoMotorSimulated::setAnalogVelocityConversionFactor(double factor) {
	;
}

void EctoMotorSimulated::configureMotionMagicSCurve(double sCurve) {
	;
}

void EctoMotorSimulated::setPercentOutput(double value) {
	; //TODO Not supported by the sim
}

double EctoMotorSimulated::getRawAnalogPosition() const {
	return 0;
}

void EctoMotorSimulated::followMotor(const EctoMotor &masterMotor, bool inverted) {
	//Not supported
	return;
}

void EctoMotorSimulated::enableVoltageCompensation(double nominalVoltage) {
	;
}

void EctoMotorSimulated::prioritizeUpdateRate() {
	;
}