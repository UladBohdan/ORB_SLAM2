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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "../../../include/Converter.h"
#include "../../../include/MapPoint.h"
#include "../../../include/System.h"

using namespace std;
using namespace ORB_SLAM2;

int file_idx = 0;
int image_idx = 1;  // pushed to ORB_SLAM as timestamps.

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){
        names = vector<string>();
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void RegisterImage(const std_msgs::String::ConstPtr& msg);

    void SaveCovisibilityGraph();

    ORB_SLAM2::System* mpSLAM;

    vector<string> names;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::NodeHandle namesHandler;
    ros::Subscriber namesSub = namesHandler.subscribe("/frames/names", 1, &ImageGrabber::RegisterImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    igb.SaveCovisibilityGraph();

    ros::shutdown();

    return 0;
}

void ImageGrabber::SaveCovisibilityGraph() {
    vector<KeyFrame*> keyframes = mpSLAM->GetCurrentKeyFrames();

    if (keyframes.empty()) {
        cout << "WARNING: no keyframes to output covisibility graph." << endl;
        return;
    }

    sort(keyframes.begin(),keyframes.end(),KeyFrame::lId);

    ofstream fout;
    fout.open("/home/parallels/co-graph/co-graph.txt");
    fout << fixed;

    for (size_t i = 0; i < keyframes.size(); i++) {
        KeyFrame* kf = keyframes[i];

        if (kf->isBad()) continue;

        fout << "KF " << (int)kf->mTimeStamp << endl << "covisible with: ";

        vector<KeyFrame*> coKFs = kf->GetCovisiblesByWeight(100);

        for (size_t j = 0; j < coKFs.size(); j++) {
            fout << (int)(coKFs[j]->mTimeStamp) << ", ";
        }
        fout << endl;

        vector<MapPoint*> mPoints(kf->GetMapPointMatches());
        fout << "number of MapPoints and KeyPoints: " << mPoints.size() << " vs. " << kf->mvKeys.size() << endl;
        fout << "MapPoint IDs: ";
        for (unsigned int j = 0; j < mPoints.size(); j++) {
            if (mPoints[j] && !mPoints[j]->isBad()) {
                fout << mPoints[j]->mnId << "(x:" << (int)(kf->mvKeys[j].pt.x) << " y:" << (int)(kf->mvKeys[j].pt.y) << "), ";
            }
        }
        fout << endl;

        fout << "======================" << endl;
    }

    fout.close();
}

// saves in TUM format.
void saveKeyFrames(vector<KeyFrame*>& keyframes) {
    sort(keyframes.begin(),keyframes.end(),KeyFrame::lId);

    if (keyframes.empty()) {
        cout << "WARNING: no keyframes to print." << endl;
        return;
    }

    ofstream f;
    char filename[50];
   	sprintf(filename, "/home/parallels/poses_temp/poses_%d_count_%lu.txt", file_idx, keyframes.size());
    file_idx++;
    f.open(filename);
    f << fixed;

    for(size_t i=0; i<keyframes.size(); i++)
    {
        KeyFrame* pKF = keyframes[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    auto cameraPose = mpSLAM->TrackMonocular(cv_ptr->image,(double)image_idx);
    image_idx++;

    cout << "========================= FRAME ====================";
    cerr << "FRAME PUSHED with ts: " << names.size() << endl;
    if (names.empty()) {
        cout << "names were not yet pushed." << endl;
    } else {
        cout << "the last name pushed:  " << names[names.size()-1] << endl;
    }
    if (cameraPose.empty()) {
        cout << "failed to estimate camera position" << endl;
    } else {
        cout << "camera pose = " << cameraPose << endl;
        // vector<KeyFrame*> keyframes = mpSLAM->GetCurrentKeyFrames();
        // saveKeyFrames(keyframes);
    }
    // cout << "number of map points: " << mpSLAM->GetTrackedMapPoints().size() << endl;
    // cout << "number of keypoints detected: " << mpSLAM->GetTrackedKeyPointsUn().size() << endl;
    // cout << "Frame calibration matrix (example): " << mpSLAM->GetTracking()->mCurrentFrame.mK << endl;
    // vector<cv::KeyPoint> kp = mpSLAM->GetTrackedKeyPointsUn();
}

void ImageGrabber::RegisterImage(const std_msgs::String::ConstPtr& msg)
{
    // cerr << names.size() << "  " << msg->data.c_str() << endl;
    names.emplace_back(string(msg->data.c_str()));
}
