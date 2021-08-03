//
// Created by lima on 21-1-12.
//

/*
 * @file CollisionShake.cpp
 * @brief Collosion logic for shaking
 *
 * */

#include "Collision/CollisionShake.h"
template <typename T>

bool CollisionShake<T>::ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                       Mat3<T>& cp_frame)
{
    int _time = this->insideTime;
    bool result = _boxs[_time % (int)(2*_period)].ContactDetection(cp_pos,penetration,cp_frame);
    return result;
}

template class CollisionShake<double>;
