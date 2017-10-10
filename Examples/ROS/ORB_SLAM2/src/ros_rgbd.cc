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


#include<algorithm>
#include<fstream>
#include<chrono>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"System.h"
//SYSTEM

#define KEYCODE_SPACE 0x20
using namespace std;
using namespace cv;
int  saveimagecount=1;
   char filename[100];

Mat dcm2quat(Mat dcm );
int sign (float i);

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void saveImage();
    ORB_SLAM2::System* mpSLAM;
    cv::Mat currentPose ;
    cv::Mat currentrgb;
    cv::Mat currentdepth;
    int imagecounter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);
    igb.imagecounter=1;

    ros::NodeHandle nh;

//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_color", 1);
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect", 1);
    

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    puts("Type any key to shutdown and save the map");
    puts("---------------------------");

    cout<<"record start "<<endl;
    cout<<"quaterion translation "<<endl;
    char filename_pose[] = "/home/mj/yaskawa_ws/image/pose.txt";
    ofstream fout(filename_pose);

    ros::Rate loop(60);
    bool iscontinue=true;

    while(iscontinue)
    {
        ros::spinOnce();
        loop.sleep();

        if( kbhit())
        {
        printf(" '%c'!/n", getchar());


        igb.saveImage();
        cv::Mat currentpose = igb.currentPose;
        cout<<"current pose : "<<endl;
        cout<<currentpose<<endl;
        Mat dcm = currentpose(Range(0,3),Range(0,3));
        cout<<dcm<<endl;
        Mat quat = dcm2quat(dcm);
        cout<<quat<<endl;
        cout<<quat.at<float>(0,0)<<endl;
        cout<<quat.at<float>(0,0)<<" "<<quat.at<float>(0,1)<<" "<<quat.at<float>(0,2)<<" "<<quat.at<float>(0,3)<<" ";
        cout<<currentpose.at<float>(0,3)<<" "<<currentpose.at<float>(1,3)<<" "<<currentpose.at<float>(2,3)<<endl;
        fout<<quat.at<float>(0,0)<<" "<<quat.at<float>(0,1)<<" "<<quat.at<float>(0,2)<<" "<<quat.at<float>(0,3)<<" ";
        fout<<currentpose.at<float>(0,3)<<" "<<currentpose.at<float>(1,3)<<" "<<currentpose.at<float>(2,3)<<endl;

        }

    }

      // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    currentPose =  mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());




//    cvtColor(cv_ptrRGB->image, currentrgb, CV_BGR2RGB);
    // sprintf(filename,"./image/rgb%d.png",saveimagecount);
    // imwrite(filename,cv_ptrRGB->image);
    // sprintf(filename,"./image/depth%d.png",saveimagecount);
    // imwrite(filename,cv_ptrD->image);
    // saveimagecount++;


//    currentrgb = cv_ptrRGB->image;
    currentdepth = cv_ptrD->image;
}
void ImageGrabber::saveImage()
{
    char rgbfilename[100];
    char depthfilename[100];
    sprintf(rgbfilename,"/home/mj/yaskawa_ws/image/rgb%d.png",imagecounter);
    sprintf(depthfilename,"/home/mj/yaskawa_ws/image/depth%d.png",imagecounter);
    imwrite(depthfilename,currentdepth);
    imwrite(rgbfilename,currentrgb);
    cout<<"save images "<<endl;
    cout<< "counter : " <<imagecounter<<endl;
    imagecounter++;
}

Mat dcm2quat(Mat dcm )
{
    Mat qnb(1,4,CV_32F);
        float temp11=dcm.at<float>(0,0);
        float temp12=dcm.at<float>(0,1);
        float temp13=dcm.at<float>(0,2);
        float temp21=dcm.at<float>(1,0);
        float temp22=dcm.at<float>(1,1);
        float temp23=dcm.at<float>(1,2);
        float temp31=dcm.at<float>(2,0);
        float temp32=dcm.at<float>(2,1);
        float temp33=dcm.at<float>(2,2);
      qnb.at<float>(0,0)=sqrt(abs(1.0 + temp11 + temp22 + temp33))/2.0;
      qnb.at<float>(0,1)=-1.0*sign(temp32-temp23) * sqrt(abs(1.0 + temp11 - temp22 - temp33))/2.0;
      qnb.at<float>(0,2)=-1.0*sign(temp13-temp31) * sqrt(abs(1.0 - temp11 + temp22 - temp33))/2.0;
      qnb.at<float>(0,3)=-1.0*sign(temp21-temp12) * sqrt(abs(1.0 - temp11 - temp22 + temp33))/2.0;
      return qnb;
}
int sign (float i)
{  if(i>0)
        return 1;
    else if(i<0)
        return -1;
    else
        return 0;
 }

