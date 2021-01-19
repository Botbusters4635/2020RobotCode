//
// Created by abiel on 1/4/20.
//

#ifndef BOTBUSTERSREBIRTH_FOLLOWPATHHEADINGCONFIG_H
#define BOTBUSTERSREBIRTH_FOLLOWPATHHEADINGCONFIG_H

#include <Control/PathFollowers/Holonomic/PIDHolonomicPathFollower.h>
#include <Control/MotionProfiles/MotionProfile.h>

struct FollowPathHeadingConfig {
	double lookaheadDistance = 0.1;
	double finishThreshold = 0.01;
	
	PIDHolonomicPathFollowerConfig pathFollowerConfig;
	
	MotionProfileConfig profileConfig;
};
#endif //BOTBUSTERSREBIRTH_FOLLOWPATHHEADINGCONFIG_H
