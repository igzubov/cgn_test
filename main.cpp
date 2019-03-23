#include <iostream>
#include <unistd.h>
#include "b0RemoteApi.h"
#include "ackermann_car.h"


class PathFollower {
public:
    PathFollower(b0RemoteApi *_client) : car(_client, "nakedAckermannSteeringCar") {
        client = _client;
    }

    void start() {
        // why boost not bind
        auto topic = client->simxDefaultSubscriber(boost::bind(&PathFollower::simTimeCallback, this, _1));
        client->simxGetSimulationStepStarted(topic);


        // auto res = client->simxSetFloatSignal("angleSignal", 30, client->simxDefaultPublisher());
        // auto res = client->simxSetIntParameter("desiredSteeringAngle", 30, client->simxDefaultPublisher());

        client->simxSpin();
    }
private:
    b0RemoteApi *client;
    AckermannCar car;

    void simTimeCallback(std::vector<msgpack::object>* msg) {
        std::map<std::string,msgpack::object> data=msg->at(1).as<std::map<std::string,msgpack::object>>();
        auto it=data.find("simulationTime");
        if (it!=data.end()) {
            // std::cout << it->second.as<float>() << std::endl;
        }
    }
};



int main() {
    PathFollower pf(new b0RemoteApi());
    pf.start();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}

