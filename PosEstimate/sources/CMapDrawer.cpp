#include "CMapDrawer.h"
#include <pangolin/pangolin.h>
#include <mutex>
using namespace std;
MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : mpMap(pMap)
{
    mKeyFrameSize = 0.5;
    mKeyFrameLineWidth = 1;
    mGraphLineWidth = 2;
    mPointSize = 5;
    mCameraSize = 0.8;
    mCameraLineWidth = 8;
}

void MapDrawer::DrawIMUPose()
{
    const std::vector<Eigen::Vector3d> &imudatas = mpMap->getIMUPose();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (size_t i = 0; i < imudatas.size(); ++i)
    {
        glVertex3f(static_cast<float>(imudatas[i].x()),
                   static_cast<float>(imudatas[i].y()),
                   static_cast<float>(imudatas[i].z()));
    }
    glEnd();
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
    //    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    //    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    cout << "render : " << vpMPs.size() << endl;
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
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

void MapDrawer::DrawRealKeyFrames()
{
    const vector<KeyFrame *> vpKFs = mpMap->GetAllRealKeyFrames();
    glLineWidth(mGraphLineWidth);
    glColor4f(0.0f, 0.0f, 1.0f, 0.6f);
    glBegin(GL_LINES);

    for (size_t i = 1; i < vpKFs.size(); i++)
    {
        KeyFrame *pPreKF = vpKFs[i - 1];
        KeyFrame *pCurKF = vpKFs[i];

        cv::Point3f prept(pPreKF->GetWorldCenter());
        glVertex3f(prept.x, prept.y, prept.z);
        cv::Point3f curpt(pCurKF->GetWorldCenter());
        glVertex3f(curpt.x, curpt.y, curpt.z);
    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

    if (bDrawKF)
    {
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetWorldPosInv().t();

            glPushMatrix();

            glMultMatrixd(Twc.ptr<double>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();
        }
    }

    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        for (size_t i = 1; i < vpKFs.size(); i++)
        {
            KeyFrame *pPreKF = vpKFs[i - 1];
            KeyFrame *pCurKF = vpKFs[i];

            cv::Point3f prept(pPreKF->GetWorldCenter());
            glVertex3f(prept.x, prept.y, prept.z);
            cv::Point3f curpt(pCurKF->GetWorldCenter());
            glVertex3f(curpt.x, curpt.y, curpt.z);
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

    glMultMatrixd(Twc.m);

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
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
    if (!mCameraPose.empty())
    {
        cv::Mat Rwc(3, 3, CV_64F);
        cv::Mat twc(3, 1, CV_64F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
        }

        M.m[0] = (float)Rwc.at<double>(0, 0);
        M.m[1] = (float)Rwc.at<double>(1, 0);
        M.m[2] = (float)Rwc.at<double>(2, 0);
        M.m[3] = 0.0;

        M.m[4] = (float)Rwc.at<double>(0, 1);
        M.m[5] = (float)Rwc.at<double>(1, 1);
        M.m[6] = (float)Rwc.at<double>(2, 1);
        M.m[7] = 0.0;

        M.m[8] = (float)Rwc.at<double>(0, 2);
        M.m[9] = (float)Rwc.at<double>(1, 2);
        M.m[10] = (float)Rwc.at<double>(2, 2);
        M.m[11] = 0.0;

        M.m[12] = (float)twc.at<double>(0);
        M.m[13] = (float)twc.at<double>(1);
        M.m[14] = (float)twc.at<double>(2);
        M.m[15] = 1.0;
    }
    else
        M.SetIdentity();
}
