#include <iostream>
#include <fstream>
#include <unistd.h>
#include "b0RemoteApi.h"
#include "ackermannCar.h"
#include <boost/geometry/geometries/point_xy.hpp>
#include <queue>
#include "pid.h"
#include "pathFollower.h"


using namespace boost::geometry;
using Point2D = model::point<double, 2, cs::cartesian>;

class Application {
public:
    Application(b0RemoteApi *client) {
        _car = new AckermannCar(client, "nakedAckermannSteeringCar");
        _client = client;
    }

    void start() {
        PathFollower pf("points.txt");

        while (!pf._achievedPoint | !pf.isEmpty()) {
            double deltaAngle = pf.getDeltaAngle(_car->getYaw(), _car->getPosition());
            _car->setSteeringAngle(deltaAngle);
            _car->setSpeed(5);
            _client->simxSpinOnce();
            _client->simxSleep(50);
        }
        _car->setSteeringAngle(0);
        _car->setSpeed(0);
    }

private:
    b0RemoteApi *_client;
    DrivableRobot *_car;
};

int main() {
    Application app(new b0RemoteApi());
    app.start();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}

