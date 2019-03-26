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
    Application(const std::string &carName, const std::string &channelName = "b0RemoteApi") {
        _client = new b0RemoteApi("b0RemoteApi_c++Client", channelName.c_str());
        _robot = new AckermannCar(_client, carName, 2);
    }

    ~Application() {
        delete _client;
        delete _robot;
    }

    void start() {
        TrajVisualizer travis(_client);
        PathFollower pf("points.txt");
        // to get start position
        _client->simxSpinOnce();
        std::vector<float> currPos = _robot->getPosition();
        std::vector<float> prevPos = currPos;
        // draw trajectory from points
        travis.drawTargetTrajectory(currPos, pf.getPoints());
        while (!pf._achievedPoint | !pf.isEmpty()) {
            currPos = _robot->getPosition();
            double deltaAngle = pf.getDeltaAngle(_robot->getYaw(), currPos);
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
};

int main() {
    Application app("nakedAckermannSteeringCar");
    app.start();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}