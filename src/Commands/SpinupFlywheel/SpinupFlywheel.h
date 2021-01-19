//
// Created by abiel on 2/16/20.
//

#ifndef BOTBUSTERSREBIRTH_SPINUPFLYWHEEL_H
#define BOTBUSTERSREBIRTH_SPINUPFLYWHEEL_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Systems/PIDShooter/PidShooter.h"

class SpinupFlywheel : public frc2::CommandHelper<frc2::CommandBase, SpinupFlywheel> {
public:
	SpinupFlywheel(const std::shared_ptr<PIDShooter> &shooter, double targetVelocity, bool toggleHood = false);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<PIDShooter> shooter;
	double targetVelocity;
	
	bool toggleHood;
};


#endif //BOTBUSTERSREBIRTH_SPINUPFLYWHEEL_H
