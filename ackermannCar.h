#include <boost/geometry/geometries/point_xy.hpp>
#include "drivableRobot.h"

#ifndef ACKERMANNCAR_H
#define ACKERMANNCAR_H

class AckermannCar : public DrivableRobot, public std::enable_shared_from_this<AckermannCar> {
public:
    AckermannCar(std::shared_ptr<b0RemoteApi> client, const std::string name, const double leftRightWheelDist,
                 const double frontRearWheelDist, const double maxSteeringAngle, const unsigned long dim);

    // in degrees
    void setSteeringAngle(double angle);

    void setSpeed(float speed);

    void initSensors();

private:
    int _lSteerHandle = 0;
    int _rSteerHandle = 0;
    int _lMotorHandle = 0;
    int _rMotorHandle = 0;
    int _fGpsHandle = 0;
    int _rGpsHandle = 0;
    int _proxHandle = 0;
    int _stopDecision = 0;

    std::vector<float> _fGps;
    std::vector<float> _rGps;

    double _leftRightWheelDistance;
    double _frontRearWheelDistance;


    void fGpsCallback(std::vector<msgpack::object> *msg);

    void rGpsCallback(std::vector<msgpack::object> *msg);

    void proxCallback(std::vector<msgpack::object> *msg);
};

#endif //ACKERMANNCAR_H