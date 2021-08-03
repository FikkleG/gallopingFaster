#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include </home/lima/workplace/LimaCheetah/160Nm——Cheetah_SDUog_software_open/user/LIMA_test_Controller/LIMAuserParameters.h>
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"

/**
 *
 */
template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped<T>* _quadruped;
  StateEstimatorContainer<T>* _stateEstimator;
  LegController<T>* _legController;
  DesiredStateCommand<T>* _desiredStateCommand;
  RobotControlParameters* controlParameters;
  LIMAuserParameters* userParameters;
  VisualizationData* visualizationData;
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H