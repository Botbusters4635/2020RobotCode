//
// Created by abiel on 2/18/20.
//

#ifndef BOTBUSTERSREBIRTH_VISIONALIGN_H
#define BOTBUSTERSREBIRTH_VISIONALIGN_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Systems/EctoSwerve/EctoSwerve.h"

#include "Utilities/VisionAlignment/VisionAlignmentHelper.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class VisionAlign : public frc2::CommandHelper<frc2::CommandBase, VisionAlign> {
public:
	/**
	 * Aligns to set targetDistance and targetCircleAngle
	 * @param swerve
	 * @param alignHelperConfig
	 * @param targetDistance
	 * @param targetCircleAngle
	 */
	VisionAlign(const std::shared_ptr<EctoSwerve> &swerve, const VisionAlignmentConfig &alignHelperConfig,
	            double targetDistance, double targetCircleAngle);
	
	/**
	 * Aligns to a set distance and current (aproximated) circle angle
	 * @param swerve
	 * @param alignHelperConfig
	 * @param targetDistance
	 */
	VisionAlign(const std::shared_ptr<EctoSwerve> &swerve, const VisionAlignmentConfig &alignHelperConfig,
	            double targetDistance);
	
	/**
	 * Only aligns yaw
	 */
	VisionAlign(const std::shared_ptr<EctoSwerve> &swerve, const VisionAlignmentConfig &alignHelperConfig);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	double startTime;
	
	const double linearTolerance = 0.01;
	const double angularTolerance = 0.01;
	
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<VisionAlignmentHelper> visionAlignHelper;
	
	VisionAlignmentSetpoint setpoint;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
};


#endif //BOTBUSTERSREBIRTH_VISIONALIGN_H
