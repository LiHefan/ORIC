/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
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
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::DrawTruthCuboids()
{
    Eigen::MatrixXd AllEdgeIds;
	AllEdgeIds.resize(14, 2); // draw 12 edges + front
	AllEdgeIds << 1, 2, 2, 3, 3, 4, 4, 1, 5, 6, 6, 7, 7, 8, 8, 5, 1, 5, 2, 6, 3, 7, 4, 8, 1, 6, 2, 5;
	AllEdgeIds.array() -= 1;
    
    for(int i=0;i<mmatTruthCuboids.rows();++i){
        Eigen::Vector3d loc_w = Eigen::Vector3d(mmatTruthCuboids(i,0),mmatTruthCuboids(i,1),mmatTruthCuboids(i,2));
        Eigen::Vector3d dim_w = Eigen::Vector3d(mmatTruthCuboids(i,6),mmatTruthCuboids(i,7),mmatTruthCuboids(i,8));
        double roll = mmatTruthCuboids(i,3), pitch = mmatTruthCuboids(i,4), yaw = mmatTruthCuboids(i,5);
        Eigen::MatrixXd cuboid_corners(3,8);   
        Eigen::MatrixXd corners_body(3,8);
        corners_body<<1, 1, -1, -1, 1, 1, -1, -1,
                        1, -1, -1, 1, 1, -1, -1, 1,
                        1, 1, 1, 1, -1, -1, -1, -1;
        Eigen::Matrix3d scale_mat=dim_w.asDiagonal();
        Eigen::Matrix3d rot;
        double su = sin(roll);
        double cu = cos(roll);
        double sv = sin(pitch);
        double cv = cos(pitch);
        double sw = sin(yaw);
        double cw = cos(yaw);
        rot(0, 0) = cv*cw;
        rot(0, 1) = su*sv*cw - cu*sw;
        rot(0, 2) = su*sw + cu*sv*cw;
        rot(1, 0) = cv*sw;
        rot(1, 1) = cu*cw + su*sv*sw;
        rot(1, 2) = cu*sv*sw - su*cw;
        rot(2, 0) = -sv;
        rot(2, 1) = su*cv;
        rot(2, 2) = cu*cv;
        cuboid_corners = rot * scale_mat * corners_body;
        //cuboid_corners = rot * scale_mat * corners_body;
        for(size_t i=0; i<8; ++i)
        {
            cuboid_corners(0,i) +=  loc_w(0);
            cuboid_corners(1,i) +=  loc_w(1);
            cuboid_corners(2,i) +=  loc_w(2);
        }

        glLineWidth(mGraphLineWidth*2);
        glBegin(GL_LINES);
        Eigen::Vector3f box_color = Eigen::Vector3f(0, 230, 0) / 255.0;
        glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
        for(int line_id = 0; line_id < AllEdgeIds.rows(); line_id++)
        {
            glVertex3f(cuboid_corners(0, AllEdgeIds(line_id,0)), cuboid_corners(1, AllEdgeIds(line_id,0)), cuboid_corners(2, AllEdgeIds(line_id,0)));
            glVertex3f(cuboid_corners(0, AllEdgeIds(line_id,1)), cuboid_corners(1, AllEdgeIds(line_id,1)), cuboid_corners(2, AllEdgeIds(line_id,1)));
            
        }
        glEnd();
        glPopMatrix();
    }

}

void MapDrawer::DrawTruthCameraPose()
{
    if(mmatTruthPoses.rows()>0)
    {
        glLineWidth(mGraphLineWidth*2);
        glBegin(GL_LINE_STRIP);
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        for(int pt_id=0;pt_id<mmatTruthPoses.rows();pt_id++)
        {
            glVertex3f(mmatTruthPoses(pt_id,1), mmatTruthPoses(pt_id,2),mmatTruthPoses(pt_id,3));
        }
        glEnd();
    }
}

void MapDrawer::DrawMapCuboids()
{
    Eigen::MatrixXd AllEdgeIds;
	AllEdgeIds.resize(14, 2); // draw 12 edges + front
	AllEdgeIds << 1, 2, 2, 3, 3, 4, 4, 1, 5, 6, 6, 7, 7, 8, 8, 5, 1, 5, 2, 6, 3, 7, 4, 8, 1, 6, 2, 5;
	AllEdgeIds.array() -= 1;
    
    const vector<MapCuboid*>& vpMCs = mpMap->GetAllMapCuboids();
    for(size_t i=0;i<vpMCs.size();++i)
    {   
        Eigen::MatrixXd cuboid_corners(3,8);
        cuboid_corners = vpMCs[i]->GetWorldPos().compute3D_BoxCorner();
        glLineWidth(mGraphLineWidth*2);
        glBegin(GL_LINES);
        Eigen::Vector3f box_color = Eigen::Vector3f(230, 0, 0) / 255.0;
        glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
        for(int line_id = 0; line_id < AllEdgeIds.rows(); line_id++)
        {
            glVertex3f(cuboid_corners(0, AllEdgeIds(line_id,0)), cuboid_corners(1, AllEdgeIds(line_id,0)), cuboid_corners(2, AllEdgeIds(line_id,0)));
            glVertex3f(cuboid_corners(0, AllEdgeIds(line_id,1)), cuboid_corners(1, AllEdgeIds(line_id,1)), cuboid_corners(2, AllEdgeIds(line_id,1)));
            
        }
        glEnd();
        glPopMatrix();
    }
    
}

} //namespace ORB_SLAM
