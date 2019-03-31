#include <b0RemoteApi.h>
#include "ackermannCar.h"
#include <thread>

AckermannCar::AckermannCar(std::shared_ptr<b0RemoteApi> client, const std::string name, const double leftRightWheelDist,
                           const double frontRearWheelDist, const double maxSteeringAngle, const unsigned long dim)
        : DrivableRobot(client, name, maxSteeringAngle, dim), _leftRightWheelDistance(leftRightWheelDist),
          _frontRearWheelDistance(frontRearWheelDist), _fGps(dim), _rGps(dim) {
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
    answ = _client->simxGetObjectHandle("prox_sensor", _client->simxServiceCall());
    _proxHandle = b0RemoteApi::readInt(answ, 1);

    _maxTurnRadius = frontRearWheelDist / sin(_maxSteeringAngle);

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
    double steeringAngleLeft = atan(
            _frontRearWheelDistance / (-(_leftRightWheelDistance / 2) + _frontRearWheelDistance / tan(radAngle)));
    double steeringAngleRight = atan(
            _frontRearWheelDistance / ((_leftRightWheelDistance / 2) + _frontRearWheelDistance / tan(radAngle)));

    _client->simxSetJointTargetPosition(_lSteerHandle, steeringAngleLeft, _client->simxDefaultPublisher());
    _client->simxSetJointTargetPosition(_rSteerHandle, steeringAngleRight, _client->simxDefaultPublisher());
}

void AckermannCar::setSpeed(float speed) {
    if (_stopDecision) {
        speed = 0;
    }
    _client->simxSetJointTargetVelocity(_lMotorHandle, speed, _client->simxDefaultPublisher());
    _client->simxSetJointTargetVelocity(_rMotorHandle, speed, _client->simxDefaultPublisher());
}

void AckermannCar::initSensors() {
    auto topic = _client->simxCreateSubscriber(boost::bind(&AckermannCar::fGpsCallback, shared_from_this(), _1));
    _client->simxGetObjectPosition(_fGpsHandle, -1, topic);
    topic = _client->simxCreateSubscriber(boost::bind(&AckermannCar::rGpsCallback, shared_from_this(), _1));
    _client->simxGetObjectPosition(_rGpsHandle, -1, topic);
    topic = _client->simxCreateSubscriber(boost::bind(&AckermannCar::proxCallback, shared_from_this(), _1));
    _client->simxCheckProximitySensor(_proxHandle, "sim.handle_all", topic);
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
    b0RemoteApi::readFloatArray(msg, _rGps, 1);
}

void AckermannCar::proxCallback(std::vector<msgpack::object> *msg) {
    _stopDecision = b0RemoteApi::readInt(msg, 1);
}