#include "pathFollower.h"

PathFollower::PathFollower(const std::string &fileName) : _pointsStream(fileName.c_str(), std::ios_base::in) {
    readPoints(_wayPoints);
}


bool PathFollower::isEmpty() {
    return _wayPoints.empty();
}

void PathFollower::readPoints(std::queue<std::vector<float>> &points) {
    int num = 0;
    _pointsStream >> num;
    for (int i = 0; i < num; i++) {
        std::vector<float> point(2);
        _pointsStream >> point[0] >> point[1];
        points.push(point);
    }
}

void PathFollower::checkPointAchievement(std::vector<float> currPos) {
    if (fabs(currPos[0] - _currTarget[0]) <= _achieveXEps &
        fabs(currPos[1] - _currTarget[1]) <= _achieveYEps) {
        _achievedPoint = true;
        _prevTarget = _currTarget;
        _wayPoints.pop();
        std::cout << "Point achieved!" << std::endl;
    }
}

double PathFollower::getDeltaAngle(double yaw, std::vector<float> currPos) {
    if (_achievedPoint) {
        _currTarget = _wayPoints.front();
        _achievedPoint = false;
        std::cout << "Point is " << _currTarget[0] << " " << _currTarget[1] << std::endl;
    }

    _prevPos = currPos;
    checkPointAchievement(currPos);

    double deltaX = _currTarget[0] - currPos[0];
    double deltaY = _currTarget[1] - currPos[1];

    // angle between object and target
    double angle = yaw - atan2(deltaX, deltaY) * 180 / M_PI;
    cutAngle(angle);
    // std::cout << "difference is " << angle << std::endl;
    return angle;
}

std::queue<std::vector<float>> PathFollower::getPoints() {
    return _wayPoints;
}

// make angle [-pi, pi]
double PathFollower::cutAngle(double &bigAngle) {
    if (bigAngle > 180) {
        bigAngle = 360 - bigAngle;
    } else if (bigAngle < -180) {
        bigAngle = 360 + bigAngle;
    }
}