#include <iostream>
#include <fstream>
#include <queue>
#include <unistd.h>
#include <boost/geometry/geometries/point_xy.hpp>
#include "b0RemoteApi.h"
#include "ackermannCar.h"
#include "pathFollower.h"
#include "trajVisualizer.h"


class Application {
public:
    Application(const std::string pointsFileName, const std::string carName = "nakedAckermannSteeringCar",
                const std::string channelName = "b0RemoteApi") {
        _client = new b0RemoteApi("b0RemoteApi_c++Client", channelName.c_str());
        _robot = new AckermannCar(_client, carName, 2);
        _pf = new PathFollower(pointsFileName);
    }

    ~Application() {
        delete _pf;
        delete _client;
        delete _robot;
    }

    void start() {
        TrajVisualizer travis(_client);
        // to get start position
        _client->simxSpinOnce();
        std::vector<float> currPos = _robot->getPosition();
        std::vector<float> prevPos = currPos;
        // draw trajectory from points
        travis.drawTargetTrajectory(currPos, _pf->getPoints());
        while (!_pf->_achievedPoint | !_pf->isEmpty()) {
            currPos = _robot->getPosition();
            double deltaAngle = _pf->getDeltaAngle(_robot->getYaw(), currPos);
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
    b0RemoteApi *_client;
    DrivableRobot *_robot;
    PathFollower *_pf;
};

int main(int argc, char **argv) {
    if (argc > 1) {
        std::string fileName(argv[1]);
        Application app(fileName);
        app.start();
        std::cout << "Hello, World!" << std::endl;
    }
    else {
        std::cout << "No file with points was specified, exiting.." << std:: endl;
    }
    return 0;
}