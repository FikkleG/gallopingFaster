//
// Created by root on 2021/1/5.
//

#ifndef INC_160NM_CHEETAH_SDUOG_SOFTWARE_OPEN_LIMA_CONTROLLER_H
#define INC_160NM_CHEETAH_SDUOG_SOFTWARE_OPEN_LIMA_CONTROLLER_H

#endif //INC_160NM_CHEETAH_SDUOG_SOFTWARE_OPEN_LIMA_CONTROLLER_H
#include <RobotController.h>
#include "FSM_States/ControlFSM.h"
#include "LIMAuserParameters.h"
#include "Controllers/ContactEstimator.h"

class LIMA_Controller:public RobotController{
public:
    LIMA_Controller(){};
    virtual ~LIMA_Controller(){};
    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters()
    {
        return &userParameters;
    }
    virtual void Estop(){ _controlFSM->initialize(); }


protected:
    ControlFSM<float>* _controlFSM;
    // Gait Scheduler controls the nominal contact schedule for the feet
    LIMAuserParameters userParameters;
};

