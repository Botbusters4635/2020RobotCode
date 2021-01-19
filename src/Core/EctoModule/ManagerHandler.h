//
// Created by abiel on 2/8/20.
//

#ifndef ECTOMODULE_MANAGERHANDLER_H
#define ECTOMODULE_MANAGERHANDLER_H

#include "Core/EctoModule/System.h"
#include <list>

class ManagerHandler : public System {
public:
    static ManagerHandler& getInstance() {
        static ManagerHandler instance;
        return instance;
    }

    ManagerHandler(ManagerHandler const &) = delete;
    ManagerHandler &operator=(ManagerHandler const &) = delete;

    void addManager(const std::function<void(void)> &updateFunction);

    void updateRobot() override ;

protected:
    explicit ManagerHandler() : System("ManagerRunner", false){
        ;
    };

    std::list<std::function<void(void)>> managerUpdate;
};


#endif //ECTOMODULE_MANAGERHANDLER_H
