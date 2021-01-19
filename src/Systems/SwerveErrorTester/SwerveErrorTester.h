//
// Created by abiel on 2/23/20.
//

#ifndef BOTBUSTERSREBIRTH_SWERVEERRORTESTER_H
#define BOTBUSTERSREBIRTH_SWERVEERRORTESTER_H

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "Systems/EctoSwerve/EctoSwerve.h"

#include <Core/EctoModule/System.h>

class SwerveErrorTester : public System {
public:
	SwerveErrorTester(const std::shared_ptr<EctoSwerve> &swerve);
	
	void initRobot() override;
	
	void updateRobot() override;

private:
	void updateNetworkTables();
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	std::shared_ptr<EctoSwerve> swerve;
	
	GenericSwerveValue wheelCurrentValue, steerCurrentValue, pidSteerError, pidWheelError;
	
	GenericSwerveValue minWheelCurrentValue, avgWheelCurrentValue, maxWheelCurrentValue;
	GenericSwerveValue minSteerCurrentValue, avgSteerCurrentValue, maxSteerCurrentValue;
	
	GenericSwerveValue minPIDSteerError, avgPIDSteerError, maxPIDSteerError;
	GenericSwerveValue minPIDWheelError, avgPIDWheelError, maxPIDWheelError;
};


#endif //BOTBUSTERSREBIRTH_SWERVEERRORTESTER_H
