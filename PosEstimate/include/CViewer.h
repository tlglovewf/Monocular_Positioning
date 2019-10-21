#ifndef VIEWER_H
#define VIEWER_H

#include "CFrameDrawer.h"
#include "CMapDrawer.h"

#include <mutex>

class FrameDrawer;
class MapDrawer;

class Viewer
{
public:
    Viewer(FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:

    bool Stop();

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};


#endif // VIEWER_H
	

