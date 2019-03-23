#include <b0RemoteApi.h>

class AckermannCar {
public:
    AckermannCar(b0RemoteApi *_client, const std::string &name) {
        client = _client;
        auto answ = client->simxGetObjectHandle(name.c_str(), client->simxServiceCall());
        carHandle = b0RemoteApi::readInt(answ, 1);
        answ = client->simxGetObjectHandle("nakedCar_steeringLeft", client->simxServiceCall());
        lSteerHandle = b0RemoteApi::readInt(answ, 1);
        answ = client->simxGetObjectHandle("nakedCar_steeringRight", client->simxServiceCall());
        rSteerHandle = b0RemoteApi::readInt(answ, 1);
        answ = client->simxGetObjectHandle("nakedCar_motorLeft", client->simxServiceCall());
        lMotorHandle = b0RemoteApi::readInt(answ, 1);
        answ = client->simxGetObjectHandle("nakedCar_motorRight", client->simxServiceCall());
        rMotorHandle = b0RemoteApi::readInt(answ, 1);

        auto topic_pos = client->simxCreateSubscriber(boost::bind(&AckermannCar::carPosCallback, this, _1));
        auto answ1 = client->simxGetObjectPosition(carHandle, -1, topic_pos);
    }
    void setSteeringAngle(double angle) {
        double radAngle = (angle * M_PI) / 180;
        double steeringAngleLeft = atan(l/(-d+l/tan(radAngle)));
        double steeringAngleRight = atan(l/(d+l/tan(radAngle)));

        client->simxSetJointTargetPosition(lSteerHandle, steeringAngleLeft, client->simxDefaultPublisher());
        client->simxSetJointTargetPosition(rSteerHandle, steeringAngleRight, client->simxDefaultPublisher());
    }
    void setSpeed(float speed) {

    }
private:
    b0RemoteApi *client;
    int carHandle;
    int lSteerHandle;
    int rSteerHandle;
    int lMotorHandle;
    int rMotorHandle;

    // 2*d=distance between left and right wheels
    const float d = 0.755;
    // distance between front and read wheels
    const float l = 2.5772;

    void setJointTargetPos(float pos) {

    }
    void setJointTargetVel(float speed) {

    }

    void carPosCallback(std::vector<msgpack::object>* msg) {
        std::vector<float> pos;
        b0RemoteApi::readFloatArray(msg, pos, 1);
        std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    }
};