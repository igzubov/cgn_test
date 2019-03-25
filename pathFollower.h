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
    const double achieveXEps = 1;
    const double achieveYEps = 1;

    bool isEmpty();

    void readPoints(std::queue<Point2D> &points);

    double getDeltaAngle(double yaw, Point2D currPos);

private:
    std::fstream pointsStream;
    std::queue<Point2D> wayPoints;
    Point2D _prevTarget;
    Point2D _currTarget;
    Point2D _prevPos;
    Point2D _currPos;

    void checkPointAchievement(Point2D currentPos);

    double cutAngle(double &bigAngle);

};