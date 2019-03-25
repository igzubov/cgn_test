#include "pathFollower.h"

PathFollower::PathFollower(const std::string &fileName) : _pointsStream(fileName.c_str(), std::ios_base::in) {
    readPoints(_wayPoints);
}


bool PathFollower::isEmpty() {
    return _wayPoints.empty();
}

void PathFollower::readPoints(std::queue<Point2D> &points) {
    int num = 0;
    int x = 0, y = 0;
    _pointsStream >> num;
    for (int i = 0; i < num; i++) {
        _pointsStream >> x >> y;
        Point2D point(x, y);
        points.push(point);
    }
}

void PathFollower::checkPointAchievement(Point2D currPos) {
    if (fabs(currPos.get<0>() - _currTarget.get<0>()) < _achieveXEps &
        fabs(currPos.get<1>() - _currTarget.get<1>()) < _achieveYEps) {
        _achievedPoint = true;
        _prevTarget = _currTarget;
        _wayPoints.pop();
        std::cout << "Point achieved!" << std::endl;
    }
}

double PathFollower::getDeltaAngle(double yaw, Point2D currPos) {
    if (_achievedPoint) {
        _currTarget = _wayPoints.front();
        _achievedPoint = false;
        std::cout << "Point is " << _currTarget.get<0>() << " " << _currTarget.get<0>() << std::endl;
    }

    checkPointAchievement(currPos);

    double deltaX = _currTarget.get<0>() - currPos.get<0>();
    double deltaY = _currTarget.get<1>() - currPos.get<1>();

    // angle between object and target
    double angle = yaw - atan2(deltaX, deltaY) * 180 / M_PI;
    cutAngle(angle);
    // std::cout << "difference is " << angle << std::endl;
    return angle;
}

// make angle [-pi, pi]
double PathFollower::cutAngle(double &bigAngle) {
    if (bigAngle > 180) {
        bigAngle = 360 - bigAngle;
    } else if (bigAngle < -180) {
        bigAngle = 360 + bigAngle;
    }
}

