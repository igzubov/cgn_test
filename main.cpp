#include <iostream>
#include <fstream>
#include <queue>
#include <unistd.h>
#include <stdexcept>
#include <boost/geometry/geometries/point_xy.hpp>
#include "b0RemoteApi.h"
#include "ackermannCar.h"
#include "pathFollower.h"
#include "trajVisualizer.h"


class Application {
public:
    Application(const std::string pointsFileName, const std::string carName = "nakedAckermannSteeringCar",
                const std::string channelName = "b0RemoteApi") : _pointsFileName(pointsFileName) {
        _client = std::make_shared<b0RemoteApi>("b0RemoteApi_c++Client", channelName.c_str());
        _robot = std::make_shared<AckermannCar>(_client, carName, 1.51, 2.5772, 45, 2);
        _robot->initSensors();
    }

    void start() {
        std::vector<std::vector<float>> points = readPoints(_pointsFileName);
        _pf = std::make_shared<PathFollower>(points);
        _pf->verifyPath();
        TrajVisualizer travis(_client);
        // to get start position
        _client->simxSpinOnce();
        std::vector<float> currPos = _robot->getPosition();
        std::vector<float> prevPos = currPos;

        // draw trajectory from points
        travis.drawTargetTrajectory(currPos, _pf->getPoints());
        while (!_pf->_achievedPoint | !_pf->isEmpty()) {
            currPos = _robot->getPosition();
            double deltaAngle = _pf->getDeltaAngle(_robot->getYaw(), currPos, _robot->getMaxTurnRadius());
            travis.drawCurrTrajectory(prevPos, currPos);
            prevPos = currPos;

            _robot->setSteeringAngle(deltaAngle);
            _robot->setSpeed(5);
            _client->simxSpinOnce();
            _client->simxSleep(50);
        }
        travis.clearAllTrajectories();
        _robot->setSteeringAngle(0);
        _robot->setSpeed(0);
    }

private:
    std::shared_ptr<b0RemoteApi> _client;
    std::shared_ptr<DrivableRobot> _robot;
    std::shared_ptr<PathFollower> _pf;
    std::string _pointsFileName;

    // read points in interval [(-48.5, 48.5) (-48.5, 48.5)]
    std::vector<std::vector<float>> readPoints(const std::string &fileName) {
        std::fstream pointsStream(fileName.c_str(), std::ios_base::in);
        int num = 0;
        std::vector<float> prevPoint(2);
        std::vector<float> currPoint(2);
        pointsStream >> num;
        // check if at least 2 points
        if (num < 2) {
            std::string errorMsg("Number of points " + std::to_string(num) + " is incorrect, need at least 2 points");
            throw std::invalid_argument(errorMsg);
        }
        std::vector<std::vector<float>> points;
        for (int i = 0; i < num; i++) {
            pointsStream >> currPoint[0] >> currPoint[1];
            // check if in map range
            if (fabs(currPoint[0]) > 48.5 || fabs(currPoint[1]) > 48.5) {
                std::string errorMsg("Point " + std::to_string(currPoint[0]) + " " + std::to_string(currPoint[1]) +
                                     " is out of range");
                throw std::invalid_argument(errorMsg);
            }
                // check if prev and curr points are not the same
            else if (!points.empty() && prevPoint == currPoint) {
                std::string errorMsg("Point " + std::to_string(currPoint[0]) + " " + std::to_string(currPoint[1]) +
                                     " is already present");
                throw std::invalid_argument(errorMsg);
            }
            points.push_back(currPoint);
            prevPoint = currPoint;
        }
        return points;
    }
};

int main(int argc, char **argv) {
    if (argc > 1) {
        std::string fileName(argv[1]);
        try {
            Application app(fileName);
            app.start();
        }
        catch (std::invalid_argument &e) {
            std::cout << e.what();
            exit(1);
        }
        std::cout << "Hello, World!" << std::endl;
    } else {
        std::cout << "No file with points was specified, exiting.." << std::endl;
    }
    return 0;
}