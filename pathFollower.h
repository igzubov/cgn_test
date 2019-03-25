#include <unistd.h>
#include <queue>
#include <fstream>
#include <boost/geometry/geometries/point_xy.hpp>
#include <iostream>

using namespace boost::geometry;
using Point2D = model::point<double, 2, cs::cartesian>;

class PathFollower {
public:
    PathFollower(const std::string &fileName);

    bool _achievedPoint = true;
    const double _achieveXEps = 1;
    const double _achieveYEps = 1;

    bool isEmpty();


    double getDeltaAngle(double yaw, Point2D currPos);

private:
    std::fstream _pointsStream;
    std::queue<Point2D> _wayPoints;
    Point2D _prevTarget;
    Point2D _currTarget;
    Point2D _prevPos;
    Point2D _currPos;

    void readPoints(std::queue<Point2D> &points);

    void checkPointAchievement(Point2D currentPos);

    double cutAngle(double &bigAngle);

};
