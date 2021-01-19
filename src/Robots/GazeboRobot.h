//
// Created by abiel on 7/14/20.
//

#ifndef BOTBUSTERSREBIRTH_GAZEBOROBOT_H
#define BOTBUSTERSREBIRTH_GAZEBOROBOT_H

#include "Core/EctoRobot.h"


class GazeboRobot : public EctoRobot {
public:
	GazeboRobot();
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void teleopInit() override;
	
	void teleopUpdate() override;
};


#endif //BOTBUSTERSREBIRTH_GAZEBOROBOT_H
