#include <b0RemoteApi.h>
#include "ackermannCar.h"

AckermannCar::AckermannCar(b0RemoteApi *client, const std::string &name) : DrivableRobot(client, name) {
    auto answ = _client->simxGetObjectHandle("nakedCar_steeringLeft", _client->simxServiceCall());
    lSteerHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_steeringRight", _client->simxServiceCall());
    rSteerHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_motorLeft", _client->simxServiceCall());
    lMotorHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_motorRight", _client->simxServiceCall());
    rMotorHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("GPSF", _client->simxServiceCall());
    fGpsHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("GPSR", _client->simxServiceCall());
    rGpsHandle = b0RemoteApi::readInt(answ, 1);

    auto topic = client->simxCreateSubscriber(boost::bind(&AckermannCar::fGpsCallback, this, _1));
    _client->simxGetObjectPosition(fGpsHandle, -1, topic);
    topic = client->simxCreateSubscriber(boost::bind(&AckermannCar::rGpsCallback, this, _1));
    _client->simxGetObjectPosition(rGpsHandle, -1, topic);


    setSteeringAngle(0);
    setSpeed(0);
}


void AckermannCar::setSteeringAngle(double angle) {
    if (angle > 45) {
        angle = 45;
    } else if (angle < -45) {
        angle = -45;
    }

    double radAngle = (angle * M_PI) / 180;
    double steeringAngleLeft = atan(l / (-d + l / tan(radAngle)));
    double steeringAngleRight = atan(l / (d + l / tan(radAngle)));

    _client->simxSetJointTargetPosition(lSteerHandle, steeringAngleLeft, _client->simxDefaultPublisher());
    _client->simxSetJointTargetPosition(rSteerHandle, steeringAngleRight, _client->simxDefaultPublisher());
}

void AckermannCar::setSpeed(float speed) {
    _client->simxSetJointTargetVelocity(lMotorHandle, speed, _client->simxDefaultPublisher());
    _client->simxSetJointTargetVelocity(rMotorHandle, speed, _client->simxDefaultPublisher());
}

void AckermannCar::fGpsCallback(std::vector<msgpack::object> *msg) {
    std::vector<float> pos;
    b0RemoteApi::readFloatArray(msg, pos, 1);
    fGps.set<0>(pos[0]);
    fGps.set<1>(pos[1]);
    position = fGps;

    // calc yaw [-pi, pi]
    double deltaX = fGps.get<0>() - rGps.get<0>();
    double deltaY = fGps.get<1>() - rGps.get<1>();
    double angle = atan2(deltaX, deltaY);

    if (angle > M_PI) {
        angle = 2 * M_PI - angle;
    } else if (angle < -M_PI) {
        angle = 2 * M_PI + angle;
    }
    yaw = angle;
}

void AckermannCar::rGpsCallback(std::vector<msgpack::object> *msg) {
    std::vector<float> pos;
    b0RemoteApi::readFloatArray(msg, pos, 1);
    rGps.set<0>(pos[0]);
    rGps.set<1>(pos[1]);
}