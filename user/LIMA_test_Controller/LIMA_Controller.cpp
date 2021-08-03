//
// Created by root on 2021/1/5.
//

#include "LIMA_Controller.h"
/**
 * Initializes the Control FSM.
 */
void LIMA_Controller::initializeController()
{
    // Initialize a new ContactEstimator object
    //_contactEstimator = new ContactEstimator<double>();
    ////_contactEstimator->initialize();

    // Initializes the Control FSM with all the required data
    _controlFSM = new ControlFSM<float>(_quadruped,
                                        _stateEstimator,
                                        _legController,
                                        _desiredStateCommand,
                                        _controlParameters,
                                        _visualizationData,
                                        &userParameters);
}
void LIMA_Controller::runController()
{
    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands();

    // Run the Control FSM code
    _controlFSM->runFSM();

    /* Jpos style
    Mat3<float> kpMat;
    Mat3<float> kdMat;
    kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 20;
    kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
    //kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
    //kdMat <<  userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
    DVec<float> _lima_ini;
    static int iter(0);
    ++iter;

    if(iter < 10){
        for(int leg(0); leg<4; ++leg){
            for(int jidx(0); jidx<3; ++jidx){
                _lima_ini[3*leg+jidx] = _legController->datas[leg].q[jidx];
            }
        }
    }

    _legController->_maxTorque = 150;
    _legController->_legsEnabled = true;
    _legController->_zeroEncoders = false;
    for(int leg(0); leg<4; ++leg)
    {
        for(int jidx(0); jidx<3; ++jidx)
        {
            float pos = std::sin(.001f * iter);
            _legController->commands[leg].qDes[jidx] = pos;
            _legController->commands[leg].qdDes[jidx] = 0.;
            _legController->commands[leg].tauFeedForward[jidx] = 10;
        }
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
    }
    */
}

