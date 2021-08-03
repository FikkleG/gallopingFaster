/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>

Quadruped<T> buildMiniCheetah() {
    Quadruped<T> cheetah;
    cheetah._robotType = RobotType::MINI_CHEETAH;

    cheetah._bodyMass = 12;
    cheetah._bodyLength = 0.287*2;
    cheetah._bodyWidth =0.0625*2;
    cheetah._bodyHeight = 0.0585*2;
    cheetah._abadGearRatio = 9;
    cheetah._hipGearRatio = 9;
    cheetah._kneeGearRatio = 9;
    cheetah._abadLinkLength = 0.107;
    cheetah._hipLinkLength = 0.3017;
    cheetah._kneeLinkY_offset =0;
    cheetah._kneeLinkLength = 0.3066;
    cheetah._maxLegLength = 0.58;


    cheetah._motorTauMax = 16.f;
    cheetah._batteryV = 42;
    cheetah._motorKT = 0.24;// this is flux linkage * pole pairs
    cheetah._motorR = 0.075;
    cheetah._jointDamping = 0.1;
    cheetah._jointDryFriction = 0.5;



    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 371, 0, 0, 0, 371, 0, 0, 0, 713;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaX =
            RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<T> rotorRotationalInertiaY =
            RX * rotorRotationalInertiaZ * RX.transpose();


    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 1402, 0, 0, 0, 2174, 0, 0, 0, 1401;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0, 0.05, 0);  // LEFT

    SpatialInertia<T> abadInertia(1.3, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 14738, 222, 877, 222, 15265, 1822, 877, 1822, 2597;
    hipRotationalInertia = hipRotationalInertia * 1e-6;

    Vec3<T> hipCOM(-0.004, 0.027, -0.034);
    SpatialInertia<T> hipInertia(2.0, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 4449, 0, -366, 0, 4501 , 0, -366, 0, 125;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0.013, 0, -0.135);
    SpatialInertia<T> kneeInertia(0.33, kneeCOM, kneeRotationalInertia);

    Vec3<T> rotorCOM(0, 0, 0);
    SpatialInertia<T> rotorInertiaX(0.36, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(0.36, rotorCOM, rotorRotationalInertiaY);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 33333, 0, 0, 0, 215733, 0, 0, 0, 241333;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(0, 0, 0);
    SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                  bodyRotationalInertia);

    cheetah._abadInertia = abadInertia;
    cheetah._hipInertia = hipInertia;
    cheetah._kneeInertia = kneeInertia;
    cheetah._abadRotorInertia = rotorInertiaX;
    cheetah._hipRotorInertia = rotorInertiaY;
    cheetah._kneeRotorInertia = rotorInertiaY;
    cheetah._bodyInertia = bodyInertia;

    // locations
    cheetah._abadRotorLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
    cheetah._abadLocation =
            Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
    cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
    cheetah._hipRotorLocation = Vec3<T>(0, 0.05, 0);
    cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
    cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

    return cheetah;
}
#endif  // PROJECT_MINICHEETAH_H
