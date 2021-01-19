//
// Created by abiel on 1/4/20.
//

#ifndef ECTOCONTROL_PATHFOLLOWER_H
#define ECTOCONTROL_PATHFOLLOWER_H

#include <Math/DataTypes/RobotPose2D.h>
#include <Math/DataTypes/Twist2D.h>

class PathFollower {
public:
    virtual Twist2D update(const RobotPose2D &currentPose) = 0;
};

#endif //ECTOCONTROL_PATHFOLLOWER_H
