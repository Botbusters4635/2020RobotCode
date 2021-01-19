//
// Created by abiel on 2/21/20.
//

#ifndef BOTBUSTERSREBIRTH_BASICAUTO_H
#define BOTBUSTERSREBIRTH_BASICAUTO_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc2/command/CommandBase.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/BallIntake/BallIntake.h"
#include "Systems/PIDShooter/PidShooter.h"

#include "Commands/WpiTrajectoryFollower/WpiTrajectoryFollower.h"
#include "Commands/SpinupFlywheel/SpinupFlywheel.h"
#include "Commands/FeedBalls/FeedBalls.h"
#include "Commands/VisionAlign/VisionAlign.h"
#include "Commands/LowerIntake/LowerIntake.h"
#include "Commands/SpinupFlywheel/SpinupFlywheel.h"
#include "Commands/FeedBalls/FeedBalls.h"
#include "Commands/PreloadBall/PreloadBall.h"

#include <Core/EctoModule/System.h>

/**
 * Base auto,
 * Starts on the right side of the field, shoots 2 balls grabs the 3 balls nearest to the Control Panel and
 * returns to shoot
 * The embodiment of pain and suffering
 * This could be much more configurable
 */
class BasicAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, BasicAuto> {
public:
	BasicAuto(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<BallFeeder> &ballFeeder,
	          const std::shared_ptr<BallIntake> &ballIntake, const std::shared_ptr<PIDShooter> &shooter);

private:
	const std::string startToShootPath = "Paths/output/StartToShoot.wpilib.json";
	const std::string goToTrenchPath = "Paths/output/GoToTrench.wpilib.json";
	const std::string grabTrenchPath = "Paths/output/GrabTrench.wpilib.json";
	const std::string shootAfterTrenchPath = "Paths/output/ShootAfterTrench.wpilib.json";
};


#endif //BOTBUSTERSREBIRTH_BASIC5BALL_H
