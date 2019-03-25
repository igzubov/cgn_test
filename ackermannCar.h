#include <boost/geometry/geometries/point_xy.hpp>
#include "drivableRobot.h"

using namespace boost::geometry;
using Point2D = model::point<double, 2, cs::cartesian>;

class AckermannCar : public DrivableRobot {
public:
    AckermannCar(b0RemoteApi *client, const std::string &name);

    // in degrees
    void setSteeringAngle(double angle);

    void setSpeed(float speed);

private:
    int _lSteerHandle;
    int _rSteerHandle;
    int _lMotorHandle;
    int _rMotorHandle;
    int _fGpsHandle;
    int _rGpsHandle;

    Point2D _fGps;
    Point2D _rGps;
    // 2*_d=distance between left and right wheels
    const float _d = 0.755;
    // distance between front and read wheels
    const float _l = 2.5772;

    void fGpsCallback(std::vector<msgpack::object> *msg);

    void rGpsCallback(std::vector<msgpack::object> *msg);
};