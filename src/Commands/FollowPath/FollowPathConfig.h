//
// Created by abiel on 1/4/20.
//

#ifndef BOTBUSTERSREBIRTH_FOLLOWPATHCONFIG_H
#define BOTBUSTERSREBIRTH_FOLLOWPATHCONFIG_H

#include <Control/PathFollowers/Holonomic/PIDHolonomicPathFollower.h>

struct FollowPathConfig {
	double lookaheadDistance = 0.1;
	double finishThreshold = 0.01;
	
	double maxAcceleration = 3;
	double maxVelocity = 2.54;
	
	double maxDeacceleration = 3;
	double minVelocity = 0.1;
	
	PIDHolonomicPathFollowerConfig pathFollowerConfig;
};
#endif //BOTBUSTERSREBIRTH_FOLLOWPATHCONFIG_H
