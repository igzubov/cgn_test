#include "trajVisualizer.h"

TrajVisualizer::TrajVisualizer(b0RemoteApi *client) : _targetTrajColor{255, 0, 0}, _currTrajColor{0, 255, 0} {
    _client = client;
}

int TrajVisualizer::drawLine(const std::vector<float> &from, const std::vector<float> &to, const int *color) {
    float segments[] = {to[0], to[1], 0, from[0], from[1], 0};
    auto answ = _client->simxAddDrawingObject_segments(4, color, segments, 6, _client->simxServiceCall());
    return b0RemoteApi::readInt(answ, 1);
}

void TrajVisualizer::drawTargetTrajectory(const std::vector<float> &start, std::queue<std::vector<float>> points) {
    std::vector<float> prevPoint = start;
    std::vector<float> currPoint;

    long amount = points.size();
    int handle = 0;
    for (int i = 0; i < amount; i++) {
        currPoint = points.front();
        points.pop();
        handle = drawLine(prevPoint, currPoint, _targetTrajColor);
        _targetTrajHandles.push(handle);
        prevPoint = currPoint;
    }

}

void TrajVisualizer::drawCurrTrajectory(const std::vector<float> &from, const std::vector<float> &to) {
    int handle = 0;
    if (_currTrajHandles.size() > 200) {
        handle = _currTrajHandles.front();
        _currTrajHandles.pop();
        _client->simxRemoveDrawingObject(handle, _client->simxDefaultPublisher());
    }
    handle = drawLine(from, to, _currTrajColor);
    _currTrajHandles.push(handle);
}

void TrajVisualizer::clearAllTrajectories() {
    long amount = _currTrajHandles.size();
    for (int i = 0; i < amount; i++) {
        _client->simxRemoveDrawingObject(_currTrajHandles.front(), _client->simxDefaultPublisher());
        _currTrajHandles.pop();
    }

    amount = _targetTrajHandles.size();
    for (int i = 0; i < amount; i++) {
        _client->simxRemoveDrawingObject(_targetTrajHandles.front(), _client->simxDefaultPublisher());
        _targetTrajHandles.pop();
    }
}
