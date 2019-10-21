#include "CMapDrawer.h"
#include <pangolin/pangolin.h>
#include <mutex>
using namespace std;
MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
//    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
//
//    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
//    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
//    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
//    mPointSize = fSettings["Viewer.PointSize"];
//    mCameraSize = fSettings["Viewer.CameraSize"];
//    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    
    mKeyFrameSize = 4;
    mKeyFrameLineWidth = 2;
    mGraphLineWidth = 1;
    mPointSize = 10;
    mCameraSize = 6;
    mCameraLineWidth = 2;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
//    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

//    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

   
    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);
    
    cout << "render : " << vpMPs.size() << endl;
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
//        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
//            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        double x = pos.at<double>(0);
        double y = pos.at<double>(1);
        double z = pos.at<double>(2);
        cout << "x = " << x << ",y = " << y << ",z = " << z << endl;
//        glVertex3f((float)pos.at<double>(0),
//                   (float)pos.at<double>(1),
//                   (float)pos.at<double>(2));
        glVertex3f(z,
                   x,
                   y);
        
    }
    glEnd();

//    glPointSize(mPointSize);
//    glBegin(GL_POINTS);
//    glColor3f(1.0,0.0,0.0);

//    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
//    {
//        if((*sit)->isBad())
//            continue;
//        cv::Mat pos = (*sit)->GetWorldPos();
//        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//
//    }

//    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetWorldPosInv().t();

            glPushMatrix();

            glMultMatrixd(Twc.ptr<GLdouble>(0));
            
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(1.0f,0.0f,0.0f);
            glPointSize(mKeyFrameSize);
            glBegin(GL_POINTS);
            glVertex3f(0, 0, 0);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
//        glLineWidth(mGraphLineWidth);
//        glColor4f(0.0f,1.0f,0.0f,0.6f);
//        glBegin(GL_LINES);
//
//        for(size_t i=0; i<vpKFs.size(); i++)
//        {
//            // Covisibility Graph
//            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
//            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
//            if(!vCovKFs.empty())
//            {
//                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
//                {
//                    if((*vit)->mnId<vpKFs[i]->mnId)
//                        continue;
//                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
//                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
//                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
//                }
//            }
//
//            // Spanning tree
//            KeyFrame* pParent = vpKFs[i]->GetParent();
//            if(pParent)
//            {
//                cv::Mat Owp = pParent->GetCameraCenter();
//                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
//                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
//            }
//
//            // Loops
//            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
//            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
//            {
//                if((*sit)->mnId<vpKFs[i]->mnId)
//                    continue;
//                cv::Mat Owl = (*sit)->GetCameraCenter();
//                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
//                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
//            }
//        }
//
//        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    glPushMatrix();

    glMultMatrixd(Twc.m);

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glPointSize(mCameraSize);
    glBegin(GL_POINTS);
    glVertex3f(0, 0, 0);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
            cv::Mat Rwc(3,3,CV_64F);
            cv::Mat twc(3,1,CV_64F);
        {
            cout << "camera pose : " << endl;
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
            cout << twc << endl;
        }

        M.m[0] = Rwc.at<double>(0,0);
        M.m[1] = Rwc.at<double>(1,0);
        M.m[2] = Rwc.at<double>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<double>(0,1);
        M.m[5] = Rwc.at<double>(1,1);
        M.m[6] = Rwc.at<double>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<double>(0,2);
        M.m[9] = Rwc.at<double>(1,2);
        M.m[10] = Rwc.at<double>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<double>(0);
        M.m[13] = twc.at<double>(1);
        M.m[14] = twc.at<double>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}
