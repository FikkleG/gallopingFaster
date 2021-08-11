/*! @file IMUTypes.h
 *  @brief Data from IMUs
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cppTypes.h"
#include "cheaterState.hpp"
#include "vectornav_lcmt.hpp"
/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData
{
  Quat<float> quat;
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  void makeVectorNavData(vectornav_lcmt *msg)
  {
      for(int i=0;i<3;i++)
          accelerometer[i] = msg->a[i];
      for(int i=0;i<3;i++)
          gyro[i] = msg->w[i];
      for(int i=0;i<4;i++)
          quat[i] = msg->q[i];
  }
  void buildLCM(vectornav_lcmt *msg)
  {
      for(int i=0;i<3;i++)
          msg->a[i] = accelerometer[i];
      for(int i=0;i<3;i++)
          msg->w[i] = gyro[i];
      for(int i=0;i<4;i++)
          msg->q[i] = quat[i];
  }

};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState
{
  Quat<T> orientation;
  Vec3<T> position;
  Vec3<T> omegaBody;
  Vec3<T> vBody;
  Vec3<T> acceleration;
  void makeCheaterState(cheaterState *msg)
  {
      for(int i=0;i<4;i++)
        orientation[i] = msg->orientation[i];
      for(int i=0;i<3;i++)
          position[i] = msg->position[i];
      for(int i=0;i<3;i++)
          omegaBody[i] = msg->omegaBody[i];
      for(int i=0;i<3;i++)
          vBody[i] = msg->vBody[i];
      for(int i=0;i<3;i++)
          acceleration[i] = msg->acceleration[i];
  }
  void buildLCM(cheaterState *msg)
  {
      for(int i=0;i<4;i++)
          msg->orientation[i] = orientation[i];
      for(int i=0;i<3;i++)
          msg->position[i] = position[i];
      for(int i=0;i<3;i++)
          msg->omegaBody[i] = omegaBody[i];
      for(int i=0;i<3;i++)
          msg->vBody[i] = vBody[i];
      for(int i=0;i<3;i++)
          msg->acceleration[i] = acceleration[i];

  }
};

#endif  // PROJECT_IMUTYPES_H
