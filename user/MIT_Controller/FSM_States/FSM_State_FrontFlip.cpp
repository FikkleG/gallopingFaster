#include "FSM_State_FrontFlip.h"
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_FrontFlip<T>::FSM_State_FrontFlip(
        ControlFSMData<T>* _controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::FRONTJUMP2,"FRONTJUMP2") {
    // Set the pre controls safety checks
    this->turnOffAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;


    // Initialize GRF to 0s
    this->footFeedForwardForces = Mat34<T>::Zero();

    _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    _wbc_data = new LocomotionCtrlData<T>();

    _wbc_ctrl->setFloatingBaseWeight(1000.);
    data=_controlFSMData;
}

template <typename T>
void FSM_State_FrontFlip<T>::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Always set the gait to be standing in this state
    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;

    _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;
    _wbc_data->pBody_des=_ini_body_pos;
    if(_ini_body_pos[2] < 0.2) {
        _ini_body_pos[2] = 0.25;
    }


    for(size_t i(0); i < 4; ++i) {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }

    last_height_command = _ini_body_pos[2];

    _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;

    _wbc_data->pBody_RPY_des=_ini_body_ori_rpy;
    _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
    _count=0;
    enter_once=0;
    enter_once_2=0;
    enter_once_3=0;
    enter_once_4=0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_FrontFlip<T>::run() {
    Vec4<T> contactState;
    contactState<< 0.5, 0.5, 0.5, 0.5;
    this->_data->_stateEstimator->setContactPhase(contactState);
    BalanceStandStep();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_FrontFlip<T>::checkTransition() {
    // Get the next state
    _iter++;

    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode) {
        case K_FRONTFLIP:
            break;

        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            break;

        case K_LOCOMOTION:
            this->nextStateName = FSM_StateName::LOCOMOTION;
            break;

        case K_PASSIVE:  // normal c
            this->nextStateName = FSM_StateName::PASSIVE;
            break;

        case K_BALANCE_STAND:
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_FRONTJUMP << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }

    // Return the next state name to the FSM
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_FrontFlip<T>::transition() {
    // Switch FSM control mode
    switch (this->nextStateName) {
        case FSM_StateName::PASSIVE:  // normal
            this->transitionData.done = true;
            break;

        case FSM_StateName::BALANCE_STAND:
            this->transitionData.done = true;
            break;

        case FSM_StateName::LOCOMOTION:
            this->transitionData.done = true;
            break;

        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;


        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition"
                      << std::endl;
    }

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_FrontFlip<T>::onExit() {
    _iter = 0;
}
template <typename T>
void FSM_State_FrontFlip<T>::BalanceStandStep() {

    _count++;

    int prepare_cout=500;
    int stage1_cout=prepare_cout+100;
    int stage2_cout=stage1_cout+180;
    int stage3_cout=stage2_cout+60;
    int stage4_cout=stage3_cout+100;
    if (_count < prepare_cout)
    {
        //prepare stage: slow down the torso height
        f_fin<<0,0,-0.15;
        fin=InverseKinematics(f_fin);
        ff_force<<0,0,0;
        for(int leg=0;leg<4;leg++)
        {
            _SetJPosInterPts(_count,prepare_cout/2,leg,initial_jpos[leg],fin);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }
        printf("front flip prepair: %d\n",_count);
    } else if (_count < stage1_cout) {
        if (enter_once == 0) {
            for (size_t i(0); i < 4; ++i) {
                initial_jpos[i] = this->_data->_legController->datas[i].q;
                initial_Ppos[i] = this->_data->_legController->datas[i].p;
            }
            enter_once = 1;
        }
        //stage 1: forward torso
        f_fin<<-0.05,0,-0.2;
        fin = InverseKinematics(f_fin);
        for(int leg=0;leg<4;leg++)
        {
            _SetJPosInterPts(_count-prepare_cout,100,leg,initial_jpos[leg],fin);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }
        printf("front flip progress 1 : --%d\n",_count);
    } else if (_count < stage2_cout) {
        if (enter_once_2 == 0) {
            for (size_t i(0); i < 4; ++i) {
                initial_jpos[i] = this->_data->_legController->datas[i].q;
                initial_Ppos[i] = this->_data->_legController->datas[i].p;
            }
            enter_once_2 = 1;
        }

        float v_omega;

        Vec3<T> zero_vec3;
        zero_vec3.setZero();

        f_fin<<-0.05,0,-0.42;
        ff_force<<-0,0,-80;
        fin = InverseKinematics(f_fin);
        for(int leg=2;leg<4;leg++)
        {
            _SetJPosInterPts(_count-stage1_cout,50,leg,initial_jpos[leg],fin);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }
        v_omega = data->_stateEstimator->getResult().omegaBody[1];
        ff_force<<0,0,0;
        for(int leg=0;leg<2;leg++) {
            initial_jpos[leg][1]+=1.*v_omega*data->controlParameters->controller_dt;
            initial_jpos[leg][2]+=0.*data->controlParameters->controller_dt;
            fin=initial_jpos[leg];
            this->jointPDControl(leg, fin, zero_vec3);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }
        if(data->_stateEstimator->getResult().rpy[1]>1.4) // pitch > 80 degree
        {
            _count = stage2_cout;
            printf("finished back the body early \n");
        }
        printf("front flip progress 2 -- %d   pitch = %.2f  v_omega=%.2f\n",_count,data->_stateEstimator->getResult().rpy[1],v_omega);
    }else if (_count < stage3_cout) { //倒立蹬地
        if (enter_once_3 == 0) {
            for (size_t i(0); i < 4; ++i) {
                initial_jpos[i] = this->_data->_legController->datas[i].q;
                initial_Ppos[i] = this->_data->_legController->datas[i].p;

            }
            enter_once_3 = 1;
        }
        Vec3<T> zero_vec3;
        zero_vec3.setZero();
        ff_force<<-150,0,-0;
        for(int leg=0;leg<2;leg++){
            initial_Ppos[leg][0]+=0.02;
            initial_Ppos[leg][2]+=0.01;
            fin= InverseKinematics(initial_Ppos[leg]);
            this->jointPDControl(leg, fin, zero_vec3);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }

        f_fin<<0,0,-0.25;
        ff_force<<0,0,0;
        fin = InverseKinematics(f_fin);
        for(int leg=2;leg<4;leg++)
        {
            _SetJPosInterPts(_count-stage2_cout,stage3_cout-stage2_cout,leg,initial_jpos[leg],fin);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }
        printf("front flip progress 3 -- %d \n",_count);
    }
    else if (_count < stage4_cout) {
        if (enter_once_4 == 0) {
            for (size_t i(0); i < 4; ++i) {
                initial_jpos[i] = this->_data->_legController->datas[i].q;
                initial_Ppos[i] = this->_data->_legController->datas[i].p;
            }
            enter_once_4 = 1;
        }
        f_fin<<0,0,-0.25;
        fin = InverseKinematics(f_fin);
        for(int leg=0;leg<4;leg++)
        {
            if(leg%2==0)
                fin(0)=-0.2;
            else
                fin(0)=0.2;
        }

        for(int leg=0;leg<4;leg++)
        {
            _SetJPosInterPts(_count-stage3_cout,100,leg,initial_jpos[leg],fin);
            this->_data->_legController->commands[leg].forceFeedForward=ff_force;
        }
        printf("front flip progress 4 --%d\n",_count);
    }
    else {

        f_fin<<0,0,-0.25;
        fin = InverseKinematics(f_fin);
        kpMat << 40, 0, 0, 0, 20, 0, 0, 0, 20;
        kdMat << 3, 0, 0, 0, 3, 0, 0, 0, 3;
        for(int leg=0;leg<4;leg++)
        {
            if(leg%2==0)
                fin(0)=-0.2;
            else
                fin(0)=0.2;
            this->_data->_legController->commands[leg].qDes = fin;
            this->_data->_legController->commands[leg].kpJoint = kpMat;
            this->_data->_legController->commands[leg].kdJoint = kdMat;
        }
        printf("front flip progress 5\n");
    }

}
// only for the smallest robot
template <typename T>
Vec3<T> FSM_State_FrontFlip<T>::InverseKinematics(Vec3<T> pos)
{
    Vec3<T> ang;
    float hipx=pos(0);
//    double hipy=pos(1);
    float hipz=pos(2);
    float L1=0.209;
    float L2=0.195;
    float PI=3.1415926;
    ang(0)=0;
    float L12=sqrt((hipz)*(hipz)+hipx*hipx);
    if(L12>0.38) L12=0.38;
    else if(L12<0.06) L12=0.06;

    float fai=acos((L1*L1+L12*L12-L2*L2)/2.0/L1/L12);
    if(isnan(fai))
    { printf("IK:fai is nan :%.2f\t%.2f\n",L1*L1+L12*L12-L2*L2,(L1*L1+L12*L12-L2*L2)/2.0/L1/L12);}
//    printf("IK:fai:%.2f\n",fai);
    ang(1)=atan2(hipx,-hipz)-fai;
    ang(2)=PI-acos((L1*L1+L2*L2-L12*L12)/2.0/L1/L2);
    return ang;
}
template <typename T>
void FSM_State_FrontFlip<T>::_SetJPosInterPts(
        const size_t & curr_iter, size_t max_iter, int leg,
        const Vec3<T> & inip, const Vec3<T> & finp){

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if(curr_iter <= max_iter) {
        b = (float)curr_iter/(float)max_iter;
        a = 1.f - b;
    }

    // compute setpoints
    Vec3<T> inter_pos = a * inip + b * finp;

    Vec3<T> zero_vec3;
    zero_vec3.setZero();
    this->jointPDControl(leg, inter_pos, zero_vec3);
}
template class FSM_State_FrontFlip<float>;