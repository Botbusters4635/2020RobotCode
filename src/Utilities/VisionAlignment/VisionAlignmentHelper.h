//
// Created by abiel on 2/17/20.
//

#ifndef BOTBUSTERSREBIRTH_VISIONALIGNMENTHELPER_H
#define BOTBUSTERSREBIRTH_VISIONALIGNMENTHELPER_H

#include "Core/VisionManager/VisionManager.h"
#include <Math/DataTypes/Point2D.h>
#include <Math/DataTypes/Twist2D.h>

#include <frc/controller/PIDController.h>
#include <frc/Notifier.h>

#include <optional>

#include <Control/EctoPID/PIDConfig.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "Core/LEDManager/LedManager.h"

struct VisionAlignmentSetpoint {
	double desiredDistance;
	double desiredCircleAngle;
};

struct VisionAlignmentConfig {
	PIDConfig thetaPIDConfig, vXPIDConfig, vYPIDConfig;
};

/**
 * Camera located at 0,0
 */
class VisionAlignmentHelper {
public:
	void enable();
	
	void disable();
	
	explicit VisionAlignmentHelper(const VisionAlignmentConfig &config);
	
	/**
	 * For latency compensation
	 * @param currentVelocity
	 */
	void setChassisVelocity(const Twist2D &currentVelocity);
	
	void changeSetpoint(const VisionAlignmentSetpoint &newSetpoint);
	
	Twist2D getCurrentOutput() const;
	
	double getCurrentDistance() const;
	
	double getAngleToTarget() const;
	
	bool isTargetDetected() const;
	
	double getXError() const;
	
	double getYError() const;
	
	double getThetaError() const;
	
	bool isThetaAtSetpoint() const;
	
	void reset();

private:
	VisionManager &visionManager = VisionManager::getInstance();
	LEDManager &manager = LEDManager::getInstance();
	
	VisionAlignmentConfig config;
	
	void update();
	
	Twist2D calculateOutput();
	
	std::unique_ptr<frc2::PIDController> thetaPID, vYPID, vXPID;
	
	mutable std::mutex outputLock;
	Twist2D currentOutput;
	
	std::unique_ptr<frc::Notifier> updateNotifier;
	
	constexpr static auto kDt = 0.01_s;
	
	mutable std::mutex toTargetMutex;
	double distanceToTarget, angleToTarget;
	
	bool isEnabled{false};
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	Twist2D currentVelocity{0, 0, 0};
	
	std::atomic<bool> targetDetected;
};


#endif //BOTBUSTERSREBIRTH_VISIONALIGNMENTHELPER_H
