/*!
 * @file Simulation.h
 * @brief Main simulation class
 */

#ifndef PROJECT_SIMULATION_H
#define PROJECT_SIMULATION_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "ControlParameters/SimulatorParameters.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimUtilities/ImuSimulator.h"
#include "SimUtilities/SimulatorMessage.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "Utilities/SharedMemory.h"
#include "Utilities/Timer.h"

#include <mutex>
#include <queue>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include "simulator_lcmt.hpp"
#include "control_t.hpp"
#include "feedBack_t.hpp"

#define SIM_LCM_NAME "simulator_state"

/*!
 * Top-level control of a simulation.
 * A simulation includes 1 robot and 1 controller
 * It does not include the graphics window: this must be set with the setWindow
 * method
 */


class Simulation
{

  friend class SimControlPanel;
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Simulation(RobotType robot, Graphics3D* window,
                      SimulatorControlParameters& params, ControlParameters& userParams,
                      std::function<void(void)> ui_update);

  /*!
   * Explicitly set the state of the robot
   */
  void setRobotState(FBModelState<double>& state){_simulator->setState(state);}

  void step(double dt, double dtLowLevelControl, double dtHighLevelControl);

  void addCollisionPlane(double mu, double resti, double height,
                         double sizeX = 20, double sizeY = 20,
                         double checkerX = 40, double checkerY = 40,
                         bool addToWindow = true);
  void addCollisionBox(double mu, double resti, double depth, double width,
                       double height, const Vec3<double>& pos,
                       const Mat3<double>& ori, bool addToWindow = true,
                       bool transparent = true);
  void addCollisonShake(double mu, double rest,
                        const Vec3<double>& pos,double period,
                        bool addToWindow=true);
  void addCollisionMesh(double mu, double resti, double grid_size,
                        const Vec3<double>& left_corner_loc,
                        const DMat<double>& height_map, bool addToWindow = true,
                        bool transparent = true);

  void lowLevelControl();

  void highLevelControl();

  /*!
   * Updates the graphics from the connected window
   */
  void updateGraphics();

  void runAtSpeed(std::function<void(std::string)> error_callback, bool graphics = true);
  void sendControlParameter(const std::string& name,
                            ControlParameterValue value,
                            ControlParameterValueKind kind,
                            bool isUser);

  void resetSimTime()
  {
    _currentSimTime = 0.;
    _timeOfNextLowLevelControl = 0.;
    _timeOfNextHighLevelControl = 0.;
  }

  ~Simulation()
  {
    delete _simulator;
    delete _robotDataSimulator;
    delete _imuSimulator;
    delete _lcm;
    _sharedMemory.closeNew();
  }

  const FBModelState<double>& getRobotState() { return _simulator->getState(); }

  void stop()
  {
    _running = false;  // kill simulation loop
    _wantStop = true;  // if we're still trying to connect, this will kill us

    if (_connected)
    {
      _sharedMemory().simToRobot.mode = SimulatorMode::EXIT;
      _sharedMemory().simulatorIsDone();
    }
  }

  SimulatorControlParameters& getSimParams() { return _simParams; }

  RobotControlParameters& getRobotParams() { return _robotParams; }
  ControlParameters& getUserParams() { return _userParams; }

  bool isRobotConnected() const { return _connected; }

  void firstRun();
  void buildLcmMessage();
  void loadTerrainFile(const std::string& terrainFileName,
                       bool addGraphics = true);
  void buildShaking()
  {
      double ori[3] = {0, -0.1, 0};
      double dangle = 0.2 / _period;
      for(int i=0; i<_period; i++)
      {
          ori[1] +=dangle;
          Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>(ori));
          R_box.transposeInPlace();
          _allOri.push_back(R_box);
      }
      for(int i=0; i<_period; i++)
      {
          ori[1] -=dangle;
          Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>(ori));
          R_box.transposeInPlace();
          _allOri.push_back(R_box);
      }
  }

    SpiCommand _spiCommand;
private:
  void handleControlError();
  Graphics3D* _window = nullptr;
  bool isvirual = false;
  //std::mutex _robotMutex;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  ImuSimulator<double>* _imuSimulator = nullptr;
  SimulatorControlParameters& _simParams;
  ControlParameters& _userParams;
  RobotControlParameters _robotParams;

  size_t _simRobotID, _controllerRobotID;
  Quadruped<double> _quadruped;
  FBModelState<double> _robotControllerState;
  FloatingBaseModel<double> _model;
  FloatingBaseModel<double> _robotDataModel;
  DVec<double> _tau;
  DynamicsSimulator<double>* _simulator = nullptr;
  DynamicsSimulator<double>* _robotDataSimulator = nullptr;
  std::vector<ActuatorModel<double>> _actuatorModels;
    SpiData _spiData;
  SpineBoard _spineBoards[4];
  TI_BoardControl _tiBoards[4];
  RobotType _robot;
  lcm::LCM* _lcm = nullptr;

  std::function<void(void)> _uiUpdate;
  std::function<void(std::string)> _errorCallback;
  bool _isShaking = false;
  u64 _time = 0;
  Vec3<double> _pos;
  int _period = 0;
  bool _running = false;
  bool _connected = false;
  bool _wantStop = false;
  vector<Mat3<double>> _allOri;
  double _desiredSimSpeed = 1.;
  double _currentSimTime = 0.;
  double _timeOfNextLowLevelControl = 0.;
  double _timeOfNextHighLevelControl = 0.;
  s64 _highLevelIterations = 0;
  simulator_lcmt _simLCM;
};

class Handler
        {
        public:
            Simulation *sim;
            ~Handler() {}
            void handleMessage(const lcm::ReceiveBuffer* rbuf,const std::string& chan,const control_t* msg)
            {
                sim->_spiCommand.makeSpiCommand(msg->controlInfo);
            }
        };



#endif  // PROJECT_SIMULATION_H
