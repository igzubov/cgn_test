#include <b0RemoteApi.h>
#include "ackermannCar.h"
#include <thread>

AckermannCar::AckermannCar(b0RemoteApi *client, const std::string name, const unsigned long dim) : DrivableRobot(client, name, dim),
                                                                                        _fGps(dim), _rGps(dim) {
    auto answ = _client->simxGetObjectHandle("nakedCar_steeringLeft", _client->simxServiceCall());
    _lSteerHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_steeringRight", _client->simxServiceCall());
    _rSteerHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_motorLeft", _client->simxServiceCall());
    _lMotorHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("nakedCar_motorRight", _client->simxServiceCall());
    _rMotorHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("GPSF", _client->simxServiceCall());
    _fGpsHandle = b0RemoteApi::readInt(answ, 1);
    answ = _client->simxGetObjectHandle("GPSR", _client->simxServiceCall());
    _rGpsHandle = b0RemoteApi::readInt(answ, 1);

    auto topic = client->simxCreateSubscriber(boost::bind(&AckermannCar::fGpsCallback, this, _1));
    _client->simxGetObjectPosition(_fGpsHandle, -1, topic);
    topic = client->simxCreateSubscriber(boost::bind(&AckermannCar::rGpsCallback, this, _1));
    _client->simxGetObjectPosition(_rGpsHandle, -1, topic);


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
    double steeringAngleLeft = atan(_l / (-_d + _l / tan(radAngle)));
    double steeringAngleRight = atan(_l / (_d + _l / tan(radAngle)));

    _client->simxSetJointTargetPosition(_lSteerHandle, steeringAngleLeft, _client->simxDefaultPublisher());
    _client->simxSetJointTargetPosition(_rSteerHandle, steeringAngleRight, _client->simxDefaultPublisher());
}

void AckermannCar::setSpeed(float speed) {
    _client->simxSetJointTargetVelocity(_lMotorHandle, speed, _client->simxDefaultPublisher());
    _client->simxSetJointTargetVelocity(_rMotorHandle, speed, _client->simxDefaultPublisher());
}

void AckermannCar::fGpsCallback(std::vector<msgpack::object> *msg) {
    b0RemoteApi::readFloatArray(msg, _fGps, 1);
    _position = _fGps;

    // calc _yaw [-pi, pi]
    double deltaX = _fGps[0] - _rGps[0];
    double deltaY = _fGps[1] - _rGps[1];
    double angle = atan2(deltaX, deltaY);

    if (angle > M_PI) {
        angle = 2 * M_PI - angle;
    } else if (angle < -M_PI) {
        angle = 2 * M_PI + angle;
    }
    _yaw = angle;
}

void AckermannCar::rGpsCallback(std::vector<msgpack::object> *msg) {
    std::vector<float> pos;
    b0RemoteApi::readFloatArray(msg, _rGps, 1);
}