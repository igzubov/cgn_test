#include <ctime>

class PidController {
public:
    PidController(double p, double i, double d);
    void updateCoeffs(double p, double i, double d);
    void zeroize();
    double calcControl(double desVal, double currVal);

private:
    double _p;
    double _i;
    double _d;
    double integral;
    double deriv;
    double prevError;
    bool inited = false;
    std::time_t prevTime;
};