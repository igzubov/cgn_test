#include <boost/geometry/geometries/point_xy.hpp>
#include "drivableRobot.h"

#ifndef ACKERMANNCAR_H
#define ACKERMANNCAR_H

class AckermannCar : public DrivableRobot {
public:
    AckermannCar(b0RemoteApi *client, const std::string name, const unsigned long dim);

    // in degrees
    void setSteeringAngle(double angle);

    void setSpeed(float speed);

private:
    int _lSteerHandle = 0;
    int _rSteerHandle = 0;
    int _lMotorHandle = 0;
    int _rMotorHandle = 0;
    int _fGpsHandle = 0;
    int _rGpsHandle = 0;

    std::vector<float> _fGps;
    std::vector<float> _rGps;
    // 2*_d=distance between left and right wheels
    const float _d = 0.755;
    // distance between front and read wheels
    const float _l = 2.5772;

    void fGpsCallback(std::vector<msgpack::object> *msg);

    void rGpsCallback(std::vector<msgpack::object> *msg);
};

#endif //ACKERMANNCAR_H