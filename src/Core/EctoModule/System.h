//
// Created by hiram on 28/06/19.
//

#ifndef BOTBUSTERS_REBIRTH_SYSTEM_H
#define BOTBUSTERS_REBIRTH_SYSTEM_H


#include <vector>
#include <memory>
#include "Utilities/EctoSettings/EctoSettings.h"
#include "Module.h"


class System : public Module {
public:
    System(const std::string & name, bool useConfigFile = true);

    virtual void initRobot();

    virtual void updateRobot();

    virtual void initDisabled();

    virtual void updateDisabled();

};


#endif //BOTBUSTERS_REBIRTH_SYSTEM_H
