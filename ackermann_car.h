class AckermannCar {
public:
    AckermannCar(b0RemoteApi *client, const std::string &name);
    // in degrees
    void setSteeringAngle(double angle);
    void setSpeed(float speed);
private:
    b0RemoteApi *_client;
    int carHandle;
    int lSteerHandle;
    int rSteerHandle;
    int lMotorHandle;
    int rMotorHandle;

    // 2*d=distance between left and right wheels
    const float d = 0.755;
    // distance between front and read wheels
    const float l = 2.5772;

    void carPosCallback(std::vector<msgpack::object>* msg);
    void carOrientCallback(std::vector<msgpack::object>* msg);
};