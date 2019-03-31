#include <b0RemoteApi.h>
#include <queue>

#ifndef TRAJVISUALIZER_H
#define TRAJVISUALIZER_H

class TrajVisualizer {
public:
    TrajVisualizer(std::shared_ptr<b0RemoteApi> client);

    int drawLine(const std::vector<float> &from, const std::vector<float> &to, const int *color);

    void drawTargetTrajectory(const std::vector<float> &start, std::vector<std::vector<float>> points);

    void drawCurrTrajectory(const std::vector<float> &from, const std::vector<float> &to);

    void clearAllTrajectories();

private:
    std::shared_ptr<b0RemoteApi> _client;
    std::vector<int> _currTrajHandles;
    std::vector<int> _targetTrajHandles;
    long _currTrajIndex = 0;
    int _currTrajSize = 0;
    int _targetTrajColor[3];
    int _currTrajColor[3];
};

#endif //TRAJVISUALIZER_H