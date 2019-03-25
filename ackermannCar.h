#include <boost/geometry/geometries/point_xy.hpp>
#include "drivableRobot.h"

using namespace boost::geometry;
using Point2D = model::point<double, 2, cs::cartesian>;

class AckermannCar : DrivableRobot {
public:
    AckermannCar(b0RemoteApi *client, const std::string &name);
    // in degrees
    void setSteeringAngle(double angle);
    void setSpeed(float speed);
private:
    int lSteerHandle;
    int rSteerHandle;
    int lMotorHandle;
    int rMotorHandle;
    int fGpsHandle;
    int rGpsHandle;

    Point2D fGps;
    Point2D rGps;
    // 2*d=distance between left and right wheels
    const float d = 0.755;
    // distance between front and read wheels
    const float l = 2.5772;

    void fGpsCallback(std::vector<msgpack::object>* msg);
    void rGpsCallback(std::vector<msgpack::object>* msg);
};