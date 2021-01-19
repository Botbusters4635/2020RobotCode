//
// Created by abiel on 2/17/20.
//

#ifndef BOTBUSTERSREBIRTH_LOWERINTAKE_H
#define BOTBUSTERSREBIRTH_LOWERINTAKE_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Systems/BallIntake/BallIntake.h"

struct LowerIntakeSetpoint {
	double leftSetpoint{0};
	double rightSetpoint{0};
	bool lowerLeft{false};
	bool lowerRight{false};
};

class LowerIntake : public frc2::CommandHelper<frc2::CommandBase, LowerIntake> {
public:
	LowerIntake(const std::shared_ptr<BallIntake> &ballIntake, const LowerIntakeSetpoint &setpoint);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<BallIntake> ballIntake;
	LowerIntakeSetpoint setpoint;
	
	bool hasInitialzed = false;
	
};


#endif //BOTBUSTERSREBIRTH_LOWERINTAKE_H
