#include "trajVisualizer.h"

TrajVisualizer::TrajVisualizer(std::shared_ptr<b0RemoteApi> client) : _targetTrajColor{255, 0, 0},
                                                                      _currTrajColor{0, 255, 0}, _currTrajHandles(200) {
    _client = client;
}

int TrajVisualizer::drawLine(const std::vector<float> &from, const std::vector<float> &to, const int *color) {
    float segments[] = {to[0], to[1], 0, from[0], from[1], 0};
    auto answ = _client->simxAddDrawingObject_segments(4, color, segments, 6, _client->simxServiceCall());
    return b0RemoteApi::readInt(answ, 1);
}

void TrajVisualizer::drawTargetTrajectory(const std::vector<float> &start, std::vector<std::vector<float>> points) {
    std::vector<float> prevPoint = start;
    std::vector<float> currPoint;
    int handle = 0;

    for (auto point : points) {
        currPoint = point;
        handle = drawLine(prevPoint, currPoint, _targetTrajColor);
        _targetTrajHandles.push_back(handle);
        prevPoint = currPoint;
    }
}

void TrajVisualizer::drawCurrTrajectory(const std::vector<float> &from, const std::vector<float> &to) {
    int handle = 0;
    if (_currTrajSize == 200) {
        handle = _currTrajHandles[_currTrajIndex];
        _client->simxRemoveDrawingObject(handle, _client->simxDefaultPublisher());
    } else {
        _currTrajSize++;
    }
    handle = drawLine(from, to, _currTrajColor);
    _currTrajHandles[_currTrajIndex] = handle;
    _currTrajIndex++;
    if (_currTrajIndex == 200) {
        _currTrajIndex = 0;
    }
}

void TrajVisualizer::clearAllTrajectories() {
    for (auto handle : _currTrajHandles) {
        _client->simxRemoveDrawingObject(handle, _client->simxDefaultPublisher());
    }

    for (auto handle : _targetTrajHandles) {
        _client->simxRemoveDrawingObject(handle, _client->simxDefaultPublisher());
    }
}