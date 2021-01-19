//
// Created by abiel on 2/22/20.
//

#ifndef BOTBUSTERSREBIRTH_SWERVEPOSITIONTRIGGER_H
#define BOTBUSTERSREBIRTH_SWERVEPOSITIONTRIGGER_H

#include <frc2/command/button/Trigger.h>
#include "Systems/EctoSwerve/EctoSwerve.h"

/**
 * FRC Command trigger which triggers when EctoSwerve reaches a triggerPose
 */
class SwervePositionTrigger : public frc2::Trigger {
public:
	SwervePositionTrigger(std::shared_ptr<const EctoSwerve> &swerve, const RobotPose2D &triggerPose,
	                      double distanceTolerance = 0.1);
	
	bool get();

private:
	std::shared_ptr<const EctoSwerve> swerve;
	
	RobotPose2D triggerPose;
	
	double distanceTolerance;
};


#endif //BOTBUSTERSREBIRTH_SWERVEPOSITIONTRIGGER_H
