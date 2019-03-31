#include <unistd.h>
#include <queue>
#include <fstream>
#include <boost/geometry/geometries/point_xy.hpp>
#include <iostream>
#include "trajVisualizer.h"

#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

class PathFollower {
public:
    PathFollower(std::vector<std::vector<float>> points);

    bool _achievedPoint = true;
    const double _achieveXEps = 1;
    const double _achieveYEps = 1;

    bool isEmpty();

    bool verifyPath();

    double getDeltaAngle(double yaw, std::vector<float> currPos, double maxTurnRadius);

    std::vector<std::vector<float>> getPoints();

private:
    std::vector<std::vector<float>> _wayPoints;
    std::vector<float> _currTarget;
    long _currTargetNum;
    bool _isAddPoint = false;
    bool _prevAddPoint = false;

    void checkPointAchievement(std::vector<float> currPos);

    bool isPointReachable(const double &angle, const double &yaw, const double maxTurnRadius,
                          const std::vector<float> &currPos, const std::vector<float> &point);

    bool isPointReachable(const std::vector<float> &currPos);

    double cutAngle(double &bigAngle);

    double calcVectorAng(const std::vector<float> prevLine, const std::vector<float> currLine);

    void removePoints();
};

#endif //PATHFOLLOWER_H