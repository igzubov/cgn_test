#include <iostream>
#include <fstream>
#include <unistd.h>
#include "b0RemoteApi.h"
#include "ackermann_car.h"
#include <boost/geometry/geometries/point_xy.hpp>
#include <queue>


using namespace boost::geometry;

const std::string pointsFile = "points.txt";
using Point2D = model::point<double, 2, cs::cartesian>;

class PathFollower {
public:
    PathFollower(b0RemoteApi *client) : car(client, "nakedAckermannSteeringCar"), wayPoints() {
        _client = client;
    }

    void start() {

        getPoints(wayPoints);
        // why boost not bind
        auto topic = _client->simxDefaultSubscriber(boost::bind(&PathFollower::simTimeCallback, this, _1));
        _client->simxGetSimulationStepStarted(topic);

        // car.setSteeringAngle(35);


        _client->simxSpin();
    }
private:
    b0RemoteApi *_client;
    AckermannCar car;
    std::queue<Point2D> wayPoints;

    void simTimeCallback(std::vector<msgpack::object>* msg) {
        std::map<std::string,msgpack::object> data=msg->at(1).as<std::map<std::string,msgpack::object>>();
        auto it=data.find("simulationTime");
        if (it!=data.end()) {
            // std::cout << it->second.as<float>() << std::endl;
        }
    }

    void getPoints(std::queue<Point2D> &points) {
        int num = 0;
        int x = 0, y = 0;
        std::fstream pointsStream(pointsFile.c_str(), std::ios_base::in);
        pointsStream >> num;
        for (int i=0; i < num; i++){
            pointsStream >> x >> y;
            Point2D point(x, y);
            points.push(point);
        }
    }
};


int main() {
    PathFollower pf(new b0RemoteApi());
    pf.start();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}

