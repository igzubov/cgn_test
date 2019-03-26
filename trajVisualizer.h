#include <b0RemoteApi.h>
#include <queue>

#ifndef TRAJVISUALIZER_H
#define TRAJVISUALIZER_H

class TrajVisualizer {
public:
    TrajVisualizer(b0RemoteApi *client);

    int drawLine(const std::vector<float> &from, const std::vector<float> &to, const int *color);

    void drawTargetTrajectory(const std::vector<float> &start, std::queue<std::vector<float>> points);

    void drawCurrTrajectory(const std::vector<float> &from, const std::vector<float> &to);

    void clearAllTrajectories();

private:
    b0RemoteApi *_client;
    std::queue<int> _currTrajHandles;
    std::queue<int> _targetTrajHandles;
    int _targetTrajColor[3];
    int _currTrajColor[3];
};

#endif //TRAJVISUALIZER_H