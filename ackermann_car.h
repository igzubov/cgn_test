class AckermannCar {
public:
    AckermannCar(b0RemoteApi *_client, const std::string &name);
    // in degrees
    void setSteeringAngle(double angle);
    void setSpeed(float speed);
private:
    b0RemoteApi *client;
    int carHandle;
    int lSteerHandle;
    int rSteerHandle;
    int lMotorHandle;
    int rMotorHandle;

    void setJointTargetPos(float pos);
    void setJointTargetVel(float speed);
};