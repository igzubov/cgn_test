#include "pid.h"

PidController::PidController(double p, double i, double d) : _p(p), _i(i), _d(d), integral(0), deriv(0), prevTime(0),
                                                             prevError(0) {}

void PidController::updateCoeffs(double p, double i, double d) {
    _p = p;
    _i = i;
    _d = d;
}

void PidController::zeroize() {
    prevTime = 0;
    prevError = 0;
    integral = 0;
    deriv = 0;
}

double PidController::calcControl(double desVal, double currVal) {
    double error = desVal - currVal;
    double dt = 0;
    std::time_t currTime = std::time(nullptr);
    if (!inited) {
        inited = true;
    } else {
        dt = currTime - prevTime;
        integral += error * dt;
        deriv = (error - prevError) / dt;
    }

    double pTerm = _p * error;
    double iTerm = _i * integral;
    double dTerm = _d * deriv;

    prevTime = currTime;
    prevError = error;

    return pTerm + iTerm + dTerm;
}