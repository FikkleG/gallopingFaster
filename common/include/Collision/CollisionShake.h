//
// Created by lima on 21-1-12.
//

#ifndef PROJECT_COLLISIONSHAKE_H
#define PROJECT_COLLISIONSHAKE_H

#endif //PROJECT_COLLISIONSHAKE_H

#include <vector>

#include "Collision.h"
#include "Utilities/utilities.h"
#include "cppTypes.h"
#include "CollisionBox.h"
#include "Math/orientation_tools.h"

/*!
 * Class to represent shake collision
 */
template <typename T>
class CollisionShake : public Collision<T>
{
public:

    /*!
     * Construct a new collision shake
     * @param mu : coefficient of friction
     * @param restitution  : rebounding ratio (v+/v-)
     * @param position : position of the shake plane center,and its the source of shaking w.r.t the global frame
     * @param period : the time (how much dt) to finish a shaking.
     * @param dangle : the angle change each dt.
     * @param isStart : true to start Shaking, false equal a slope
     */
    CollisionShake(const T& mu, const T& restitution, const Vec3<T>& position, const T& period, const std::vector<Mat3<T>>& allOri)
            : Collision<T>(mu, restitution), _position(position),time(0), _period(period), isStart(false)
    {
        for(int i = 0; i < 2*period; i++)
            _boxs.push_back(CollisionBox<T>(mu, restitution, 5.0, 5.0, 0.1, position, allOri[i]));
    }

    virtual ~CollisionShake() {}
    virtual bool ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                  Mat3<T>& cp_frame);
    void StartShaking(){isStart = true;}
    void StopShaking(){isStart = false;};
private:
    Vec3<T> _position;
    int time;
    T _period;
    bool isStart;
    std::vector<CollisionBox<T>> _boxs;
};
