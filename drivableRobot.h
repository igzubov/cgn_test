#include <boost/geometry/geometries/point_xy.hpp>
#include "b0RemoteApi.h"

using namespace boost::geometry;
using Point2D = model::point<double, 2, cs::cartesian>;


class DrivableRobot {
public:
    DrivableRobot(b0RemoteApi *client, const std::string &name) {
        _client = client;
        auto answ = client->simxGetObjectHandle(name.c_str(), _client->simxServiceCall());
        _robotHandle = b0RemoteApi::readInt(answ, 1);
    };
    Point2D getPosition() {return _position;};
    double getYaw() {return _yaw * 180 / M_PI;};
    // in degrees
    virtual void setSteeringAngle(double angle) = 0;
    virtual void setSpeed(float speed) = 0;
protected:
    b0RemoteApi *_client;
    int _robotHandle;
    Point2D _position;
    double _yaw;
};