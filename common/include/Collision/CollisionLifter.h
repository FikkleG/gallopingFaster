//
// Created by lima on 2021/2/22.
//

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
class CollisionLifter : public Collision<T>
{
public:

    /*!
     * Construct a new collision lifter
     * @param mu : coefficient of friction
     * @param restitution  : rebounding ratio (v+/v-)
     * @param position : position of the shake plane center,and its the source of shaking w.r.t the global frame
     * @param period : the time (how much dt) to finish a shaking.
     * @param dangle : the angle change each dt.
     * @param isStart : true to start Shaking, false equal a slope
     */
    CollisionLifter(const T& mu, const T& restitution, const Vec3<T>& position, const T& period, const std::vector<T>& allHeight)
            : Collision<T>(mu, restitution), _position(position),_period(period), isStart(false)
    {
        Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>({0,0,0}));
        for(int i = 0; i < 2*period; i++)
            _boxs.push_back(CollisionBox<T>(mu, restitution, 0.2, 0.2, allHeight[i], _position, R_box));
    }

    virtual ~CollisionLifter() {}
    virtual bool ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                  Mat3<T>& cp_frame);
    void StartShaking(){};
    void StopShaking(){};
private:
    Vec3<T> _position;
    T _period;
    bool isStart;
    std::vector<CollisionBox<T>> _boxs;
};
