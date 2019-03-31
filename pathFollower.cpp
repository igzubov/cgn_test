#include "pathFollower.h"

PathFollower::PathFollower(std::vector<std::vector<float>> points) : _currTargetNum(0) {
    _wayPoints = points;
}

bool PathFollower::isEmpty() {
    return _currTargetNum == _wayPoints.size();
}

void PathFollower::checkPointAchievement(std::vector<float> currPos) {
    if (fabs(currPos[0] - _currTarget[0]) <= _achieveXEps &
        fabs(currPos[1] - _currTarget[1]) <= _achieveYEps) {
        _achievedPoint = true;
        _currTargetNum++;
        std::cout << "Point achieved!" << std::endl;
    }
}

// check angle between all lines in path
bool PathFollower::verifyPath() {
    if (_wayPoints.size() > 2) {
        std::vector<float> prevLine(2);
        std::vector<float> currLine(2);
        prevLine[0] = _wayPoints[1][0] - _wayPoints[0][0];
        prevLine[1] = _wayPoints[1][1] - _wayPoints[0][1];
        for (int i = 2; i < _wayPoints.size(); i++) {
            currLine[0] = _wayPoints[i][0] - _wayPoints[i - 1][0];
            currLine[1] = _wayPoints[i][1] - _wayPoints[i - 1][1];

            double ang = calcVectorAng(prevLine, currLine);

            if (fabs(ang) > 45) {
                std::cout << "Point " << _wayPoints[i][0] << " " << _wayPoints[i][1]
                          << " is incorrect (too big angle). Will continue but no guarantee to finish in last point"
                          << std::endl;
                return false;
            }
            prevLine = currLine;
        }
    }
    return true;
}

// get angle to steer
double PathFollower::getDeltaAngle(double yaw, std::vector<float> currPos, double maxTurnRadius) {
    if (_achievedPoint) {
        _currTarget = _wayPoints[_currTargetNum];
        _achievedPoint = false;
        if (_isAddPoint) {
            _isAddPoint = false;
            _prevAddPoint = true;
        } else {
            _prevAddPoint = false;
        }
        std::cout << "Point is " << _currTarget[0] << " " << _currTarget[1] << std::endl;
    }

    // add point in opposite direction to next two points
    if (!isPointReachable(currPos) && !_isAddPoint && !_prevAddPoint) {
        std::cout << "Point is not reachable!" << std::endl;
        double x = _wayPoints[_currTargetNum + 1][0] - _wayPoints[_currTargetNum][0];
        double y = _wayPoints[_currTargetNum + 1][1] - _wayPoints[_currTargetNum][1];
        double length = sqrt(x * x + y * y);
        double normX = x / length;
        double normY = y / length;
        _currTarget[0] = -1 * 10 * normX + _wayPoints[_currTargetNum][0];
        _currTarget[1] = -1 * 10 * normY + _wayPoints[_currTargetNum][1];
        _currTargetNum--;
        _isAddPoint = true;
    }

    checkPointAchievement(currPos);

    // check if target in the map
    if (fabs(_currTarget[0]) > 48.5 || fabs(_currTarget[1]) > 48.5) {
        std::cout << "Can't reach this point anyway, stopping.." << std::endl;
        removePoints();
    }


    double deltaX = _currTarget[0] - currPos[0];
    double deltaY = _currTarget[1] - currPos[1];

    // angle between object and target
    double an = atan2(deltaX, deltaY) * 180 / M_PI;
    if (an < 0) {
        an += 360;
    }
    double angle = yaw - an;
    cutAngle(angle);

    return angle;
}

std::vector<std::vector<float>> PathFollower::getPoints() {
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

// calc angle between two vectors
double PathFollower::calcVectorAng(const std::vector<float> prevLine, const std::vector<float> currLine) {
    double dotProd = prevLine[0] * currLine[0] + prevLine[1] * currLine[1];
    double prevMagn = sqrt(prevLine[0] * prevLine[0] + prevLine[1] * prevLine[1]);
    double currMagn = sqrt(currLine[0] * currLine[0] + currLine[1] * currLine[1]);

    return acos(dotProd / (prevMagn * currMagn)) * 180 / M_PI;
}

// check angle between vectors of current pos and next two targets
bool PathFollower::isPointReachable(const std::vector<float> &currPos) {
    if (_currTargetNum + 1 == _wayPoints.size()) {
        return true;
    }
    std::vector<float> currLine(2);
    std::vector<float> nextLine(2);
    currLine[0] = _currTarget[0] - currPos[0];
    currLine[1] = _currTarget[1] - currPos[1];
    nextLine[0] = _wayPoints[_currTargetNum + 1][0] - _currTarget[0];
    nextLine[1] = _wayPoints[_currTargetNum + 1][1] - _currTarget[1];
    double angle = fabs(calcVectorAng(currLine, nextLine));

    return angle <= 45;
}

// check if we can reach point with max steering angle
bool PathFollower::isPointReachable(const double &angle, const double &yaw, const double maxTurnRadius,
                                    const std::vector<float> &currPos, const std::vector<float> &point) {
    // coordinates of the center of circle
    double x0 = 0;
    double y0 = 0;
    if (angle > 45) {
        x0 = currPos[0] - maxTurnRadius * cos((-yaw + 45.0f) / 180 * M_PI);
        y0 = currPos[1] - maxTurnRadius * sin((-yaw + 45.0f) / 180 * M_PI);
    } else if (angle < -45) {
        x0 = currPos[0] - maxTurnRadius * cos((-yaw + (180 - 45.0f)) / 180 * M_PI);
        y0 = currPos[1] - maxTurnRadius * sin((-yaw + (180 - 45.0f)) / 180 * M_PI);
    } else {
        return true;
    }
    // check if length of vector more than max radius
    double length = sqrt(pow((point[0] - x0), 2) + pow((point[1] - y0), 2));

    return length > maxTurnRadius + 2;
}

void PathFollower::removePoints() {
    _currTargetNum = _wayPoints.size();
    _achievedPoint = true;
}