#include <b0RemoteApi.h>
#include "ackermann_car.h"

AckermannCar::AckermannCar(b0RemoteApi *client, const std::string &name) {
    _client = client;
    auto answ = client->simxGetObjectHandle(name.c_str(), _client->simxServiceCall());
    carHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_steeringLeft", _client->simxServiceCall());
    lSteerHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_steeringRight", _client->simxServiceCall());
    rSteerHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_motorLeft", _client->simxServiceCall());
    lMotorHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_motorRight", _client->simxServiceCall());
    rMotorHandle = b0RemoteApi::readInt(answ, 1);

    auto topic = client->simxCreateSubscriber(boost::bind(&AckermannCar::carPosCallback, this, _1));
    _client->simxGetObjectPosition(carHandle, -1, topic);
    topic = client->simxCreateSubscriber(boost::bind(&AckermannCar::carOrientCallback, this, _1));
    _client->simxGetObjectOrientation(carHandle, -1, topic);


    setSteeringAngle(0);
    setSpeed(0);
}

void AckermannCar::setSteeringAngle(double angle) {
    if (angle > 45) {
        angle = 45;
    }
    else if (angle < -45) {
        angle = -45;
    }

    double radAngle = (angle * M_PI) / 180;
    double steeringAngleLeft = atan(l/(-d+l/tan(radAngle)));
    double steeringAngleRight = atan(l/(d+l/tan(radAngle)));

    _client->simxSetJointTargetPosition(lSteerHandle, steeringAngleLeft, _client->simxDefaultPublisher());
    _client->simxSetJointTargetPosition(rSteerHandle, steeringAngleRight, _client->simxDefaultPublisher());
}

void AckermannCar::setSpeed(float speed) {
    _client->simxSetJointTargetVelocity(lMotorHandle, speed, _client->simxDefaultPublisher());
    _client->simxSetJointTargetVelocity(rMotorHandle, speed, _client->simxDefaultPublisher());
}

void AckermannCar::carPosCallback(std::vector<msgpack::object>* msg) {
    std::vector<float> pos;
    b0RemoteApi::readFloatArray(msg, pos, 1);
    // std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
}

void AckermannCar::carOrientCallback(std::vector<msgpack::object>* msg) {
    std::vector<float> orient;
    b0RemoteApi::readFloatArray(msg, orient, 1);
    // roll yaw pitch
    std::cout << orient[0] << " " << orient[1] << " " << orient[2] << std::endl;
}