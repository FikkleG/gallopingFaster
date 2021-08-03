//
// Created by lima on 2021/2/22.
//

#include "Collision/CollisionLifter.h"
template <typename T>

bool CollisionLifter<T>::ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                         Mat3<T>& cp_frame)
{
    int _time = this->insideTime;
    bool result = false;
    if(_time<2000)
        result = _boxs[_time].ContactDetection(cp_pos,penetration,cp_frame);
    else
        result = _boxs[(_time - 2000) % (int)(_period) + 2000].ContactDetection(cp_pos,penetration,cp_frame);
    return result;
}

template class CollisionLifter<double>;
