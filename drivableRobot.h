#include <boost/geometry/geometries/point_xy.hpp>
#include "b0RemoteApi.h"

class DrivableRobot {
public:
    DrivableRobot(std::shared_ptr<b0RemoteApi> client, const std::string name, const double maxSteeringAngle, const unsigned long dim) : _position(dim), _maxSteeringAngle(maxSteeringAngle) {
        _client = client;
        auto answ = client->simxGetObjectHandle(name.c_str(), _client->simxServiceCall());
        _robotHandle = b0RemoteApi::readInt(answ, 1);
    };

    virtual ~DrivableRobot() = default;
    std::vector<float> getPosition() {return _position;};
    double getYaw() {return _yaw * 180 / M_PI;};
    double getMaxTurnRadius() {return _maxTurnRadius;};
    // in degrees
    virtual void setSteeringAngle(double angle) = 0;
    virtual void setSpeed(float speed) = 0;
    virtual void initSensors() = 0;
protected:
    std::shared_ptr<b0RemoteApi> _client;
    int _robotHandle = 0;
    std::vector<float> _position;
    double _yaw = 0;
    double _maxTurnRadius;
    double _maxSteeringAngle;
};