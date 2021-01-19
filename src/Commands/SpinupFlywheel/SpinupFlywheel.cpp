//
// Created by abiel on 2/16/20.
//

#include "SpinupFlywheel.h"

SpinupFlywheel::SpinupFlywheel(const std::shared_ptr<PIDShooter> &shooter, double targetVelocity, bool toggleHood) {
	this->shooter = shooter;
	this->targetVelocity = targetVelocity;
	this->toggleHood = toggleHood;
	
	this->SetName("SpinupFlywheel");
}

void SpinupFlywheel::Initialize() {
	shooter->setHood(this->toggleHood ? HoodPosition::LongRange : HoodPosition::CloseRange);
	shooter->spinup(targetVelocity);
}

void SpinupFlywheel::Execute() {
	;
}

void SpinupFlywheel::End(bool interrupted) {
	;
}

bool SpinupFlywheel::IsFinished() {
	//return true;
	return shooter->isReadyToShoot();
}