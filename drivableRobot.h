#include <boost/geometry/geometries/point_xy.hpp>
#include "b0RemoteApi.h"

using namespace boost::geometry;
using Point2D = model::point<double, 2, cs::cartesian>;


class DrivableRobot {
public:
    DrivableRobot(b0RemoteApi *client, const std::string &name) {
        _client = client;
        auto answ = client->simxGetObjectHandle(name.c_str(), _client->simxServiceCall());
        robotHandle = b0RemoteApi::readInt(answ, 1);
    };
    virtual Point2D getPosition() {return position;};
    virtual double getYaw() {return yaw * 180 / M_PI;};
    // in degrees
    virtual void setSteeringAngle(double angle) = 0;
    virtual void setSpeed(float speed) = 0;
protected:
    b0RemoteApi *_client;
    int robotHandle;
    Point2D position;
    double yaw;
};