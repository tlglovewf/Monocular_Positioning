#include "CViewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <mutex>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
using namespace std;
Viewer::Viewer(FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer) : mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
                                                                   mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    mT          = 1e3 / 10.0;
    mViewpointX = 0;
    mViewpointY = -10;
    mViewpointZ = -0.1;
    mViewpointF = 400;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    static bool init = false;
    pangolin::OpenGlMatrix Twc;

    // if (!init)
    // {
        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 1024, 768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

        // Add named OpenGL viewport to window and provide 3D Handler

        Twc.SetIdentity();

        static pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        static pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        static pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        static pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        static pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        static pangolin::Var<bool> menuReset("menu.Reset", false, false);
        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 5000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

        // cv::namedWindow("ORB-SLAM2: Current Frame");
    //     init = true;
    // }

    bool bFollow = true;
    bool bLocalizationMode = false;

    while (1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if (menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if (menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if (!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        mpMapDrawer->DrawCurrentCamera(Twc);
        mpMapDrawer->DrawRealKeyFrames();
        if (menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);

        
        // if(menuShowPoints)
        // mpMapDrawer->DrawMapPoints();
        mpMapDrawer->DrawIMUPose();
        pangolin::FinishFrame();

        //        cv::Mat im = mpFrameDrawer->DrawFrame();

        //        cv::Mat o_im;
        //        cv::resize(im, o_im, cv::Size(im.cols/2.0,im.rows/2.0));
        //        cv::imshow("ORB-SLAM2: Current Frame",o_im);
        //        cv::waitKey(mT);

        if (menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            menuReset = false;
        }

        //        if(Stop())
        //        {
        //            while(isStopped())
        //            {
        //                usleep(3000);
        //            }
        //        }

        //        if(CheckFinish())
        //            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if (!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if (mbFinishRequested)
        return false;
    else if (mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}
