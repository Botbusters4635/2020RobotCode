//
// Created by abiel on 2/23/20.
//

#include "SwerveErrorTester.h"

SwerveErrorTester::SwerveErrorTester(const std::shared_ptr<EctoSwerve> &swerve) : System("SwerveErrorTester") {
	this->table = ntInstance.GetTable("SwerveErrorTester");
}

void SwerveErrorTester::initRobot() {
	;
}

void SwerveErrorTester::updateRobot() {
	wheelCurrentValue = swerve->getWheelCurrents();
	steerCurrentValue = swerve->getSteerCurrents();
	
	pidSteerError = swerve->getSteerPIDError();
	pidWheelError = swerve->getWheelPIDError();
	
	minWheelCurrentValue = GenericSwerveValue::min(minWheelCurrentValue, wheelCurrentValue);
	avgWheelCurrentValue = GenericSwerveValue::avg(minWheelCurrentValue, wheelCurrentValue);
	maxWheelCurrentValue = GenericSwerveValue::max(minWheelCurrentValue, wheelCurrentValue);
	
	minSteerCurrentValue = GenericSwerveValue::min(minSteerCurrentValue, steerCurrentValue);
	avgSteerCurrentValue = GenericSwerveValue::avg(minSteerCurrentValue, steerCurrentValue);
	maxSteerCurrentValue = GenericSwerveValue::max(minSteerCurrentValue, steerCurrentValue);
	
	minPIDSteerError = GenericSwerveValue::min(minSteerCurrentValue, steerCurrentValue);
	avgPIDSteerError = GenericSwerveValue::avg(avgPIDSteerError, steerCurrentValue);
	maxPIDSteerError = GenericSwerveValue::max(maxPIDSteerError, steerCurrentValue);
	
	minPIDWheelError = GenericSwerveValue::min(minPIDWheelError, steerCurrentValue);
	avgPIDWheelError = GenericSwerveValue::avg(avgPIDWheelError, steerCurrentValue);
	maxPIDWheelError = GenericSwerveValue::max(maxPIDWheelError, steerCurrentValue);
	
	updateNetworkTables();
}

void SwerveErrorTester::updateNetworkTables() {
	minWheelCurrentValue.publishToNT(table, "MinWheelCurrentValue");
	avgWheelCurrentValue.publishToNT(table, "AvgWheelCurrentValue");
	maxWheelCurrentValue.publishToNT(table, "MaxWheelCurrentValue");
	
	minSteerCurrentValue.publishToNT(table, "MinSteerCurrentValue");
	avgSteerCurrentValue.publishToNT(table, "AvgSteerCurrentValue");
	maxSteerCurrentValue.publishToNT(table, "MaxSteerCurrentValue");
	
	minPIDSteerError.publishToNT(table, "MinPIDSteerError");
	avgPIDSteerError.publishToNT(table, "AvgPIDSteerError");
	maxPIDSteerError.publishToNT(table, "MaxPIDSteerError");
	
	minPIDWheelError.publishToNT(table, "MinPIDWheelError");
	avgPIDWheelError.publishToNT(table, "AvgPIDWheelError");
	maxPIDWheelError.publishToNT(table, "MaxPIDWheelError");
	
	wheelCurrentValue.publishToNT(table, "WheelCurrent");
	steerCurrentValue.publishToNT(table, "SteerCurrent");
	pidSteerError.publishToNT(table, "PIDSteerError");
	pidWheelError.publishToNT(table, "PIDWheelError");
}