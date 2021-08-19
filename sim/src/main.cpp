/*! @file main.cpp
 *  @brief Main function for simulator
 */

#include "Collision/CollisionPlane.h"
#include "DrawList.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimControlPanel.h"
#include "Simulation.h"
#include "Utilities/utilities.h"
#include "Utilities/SegfaultHandler.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <stdio.h>
#include <unistd.h>
#include <thread>

/*!
 * Setup QT and run a simulation
 */


void run(Simulation *_simulation)
{

}

int main(int argc, char *argv[])
{
    install_segfault_handler(nullptr);
    RobotType robotType = RobotType::MINI_CHEETAH;
    SimulatorControlParameters _parameters;
    ControlParameters _userParameters("user-parameters");
    _parameters.initializeFromYamlFile("/media/gj/data/gallopingFaster/config/simulator-defaults.yaml");
    _userParameters.defineAndInitializeFromYamlFile(getConfigDirectoryPath() + "mc-mit-ctrl-user-parameters.yaml");
    auto *_simulation = new Simulation(robotType, _parameters, _userParameters);
    _simulation->loadTerrainFile("/media/gj/data/gallopingFaster/config/default-terrain.yaml");
    //std::thread _simThread = std::thread(run, _simulation);
    std::function<void(std::string)> error_function = [](std::string str){};
    _simulation->runAtSpeed(error_function);

    return 0;
}

/*old version
int main(int argc, char *argv[])
{
  install_segfault_handler(nullptr);
  // set up Qt
  QApplication a(argc, argv);

  // open simulator UI
  SimControlPanel panel;
  panel.show();

  // run the Qt program
  a.exec();

  return 0;
}
*/