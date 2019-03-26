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
    PathFollower(const std::string &fileName);

    bool _achievedPoint = true;
    const double _achieveXEps = 1;
    const double _achieveYEps = 1;

    bool isEmpty();


    double getDeltaAngle(double yaw, std::vector<float> currPos);

    std::queue<std::vector<float>> getPoints();

private:
    std::fstream _pointsStream;
    std::queue<std::vector<float>> _wayPoints;
    std::vector<float> _prevTarget;
    std::vector<float> _currTarget;
    std::vector<float> _prevPos;
    std::vector<float> _currPos;

    void readPoints(std::queue<std::vector<float>> &points);

    void checkPointAchievement(std::vector<float> currPos);

    double cutAngle(double &bigAngle);
};

#endif //PATHFOLLOWER_H