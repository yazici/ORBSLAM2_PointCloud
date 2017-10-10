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

//ROS

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include "setting.h"

#include"System.h"
//SYSTEM

#define KEYCODE_SPACE 0x20
using namespace std;
using namespace cv;

Mat cameraMatrix;
Mat distCoeffs;
Mat object_point;
Mat first_position;


Settings  s;
bool first_flag = true;
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
bool    initSettings(int WIDTH,int HEIGHT,float SQUARESIZE);
Mat     calcBoardCornerPositions(Size boardSize, float squareSize);
bool calculate_extrinsic_m(Mat image,Mat *transform,Mat object_point,Mat cameraMatrix,Mat distCoeffs,Settings s );
Mat doubleMat2floatMat(Mat t);
Mat dcm2quat(Mat dcm );
int sign (double i);
bool checkcurrentpose(cv::Mat t);

bool mLocalization;

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
    void pubTF();
    ORB_SLAM2::System* mpSLAM;
    cv::Mat currentPose ;
    cv::Mat currentrgb;
    cv::Mat absPose;

    tf::TransformBroadcaster br;
   tf::Transform transform;
    int imagecounter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);
    igb.imagecounter=1;

    // Init calibration setting
    initSettings(7,5,30);
    int localization = atoi(argv[3]);
    if(localization==1) mLocalization=true;
    else mLocalization  =false;



    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
//        message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
//        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);



    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    puts("Type any key to shutdown and save the map");
    puts("---------------------------");

    cout<<"record start "<<endl;
    cout<<"quaterion translation "<<endl;

    ros::Rate loop(60);
    bool iscontinue=true;

    while(iscontinue)
    {
        ros::spinOnce();
        loop.sleep();

        if( kbhit())
        {
        printf(" '%c'!/n", getchar());

        iscontinue = false;
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
    if(mLocalization == false){
        if(first_flag)
        {
            imwrite("rgb1.png",cv_ptrRGB->image);
            first_flag = false;
            Mat transform(4,4,CV_64F);
            calculate_extrinsic_m(cv_ptrRGB->image,&transform,object_point,cameraMatrix,distCoeffs,s);

            first_position =doubleMat2floatMat(transform);
            cout<<"first frame position"<<endl;
            cout<<first_position<<endl;



            FileStorage fs("firstpose.xml",FileStorage::WRITE);

            fs<<"firstpose"<<first_position;
            fs.release();
        }
    }
    else
    {
        FileStorage fs("firstpose.xml",FileStorage::READ);
        fs["firstpose"]>>first_position;
        fs.release();

    }
    Mat color ;
//    cvtColor(cv_ptrRGB->image,color,CV_RGB2BGR);

    currentPose =  mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if(checkcurrentpose(currentPose))
    {
        absPose = currentPose*first_position;
        cout<<"currentpose"<<endl;
        cout<<currentPose<<endl;

        pubTF();
    }

}
bool checkcurrentpose(cv::Mat t)
{
   if((fabs(t.at<float>(0,3))<1e-5)&&(fabs(t.at<float>(0,3))<1e-5)&&(fabs(t.at<float>(0,3))<1e-5))
       return false;
   else return true;
}
void ImageGrabber::pubTF()
{

    float t1 = absPose.at<float>(0,3);
    float t2 = absPose.at<float>(1,3);
    float t3 = absPose.at<float>(2,3);

    transform.setOrigin( tf::Vector3(t1,t2,t3) );
    Mat dcm(3,3,CV_64F);
    for (int i =0 ;i<3;i++)
    {
        for (int j = 0;j<3;j++)
        {
            dcm.at<double>(i,j) =absPose.at<float>(i,j);
        }
    }

    Mat quat=dcm2quat(dcm);
    double w,x,y,z;
    w=-quat.at<double>(0,0);
    x=quat.at<double>(0,1);
    y=quat.at<double>(0,2);
    z=quat.at<double>(0,3);
    tf::Quaternion q(x,y,z,w);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/kinect2_rgb_optical_frame","world"));




}


bool  initSettings(int WIDTH,int HEIGHT,float SQUARESIZE)
{
    s.boardSize.width=WIDTH;
    s.boardSize.height=HEIGHT;
    s.squareSize=SQUARESIZE;
    s.patternToUse=1;
    s.input=" ";
    s.flipVertical=0;
    s.delay=100;
    s.nrFrames=25;
    s.aspectRatio=1;
    s.calibZeroTangentDist=1;
    s.calibFixPrincipalPoint=1;
    s.outputFileName=" ";
    s.bwritePoints=1;
    s.bwriteExtrinsics=1;
    s.showUndistorsed=1;
    s.inputType=Settings::IMAGE_LIST;

    s.flag = 0;
    if(s.calibFixPrincipalPoint) s.flag |= CV_CALIB_FIX_PRINCIPAL_POINT;;
    if(s.calibZeroTangentDist)   s.flag |= CV_CALIB_ZERO_TANGENT_DIST;
    if(s.aspectRatio)            s.flag |= CV_CALIB_FIX_ASPECT_RATIO;

   s. calibrationPattern = Settings::CHESSBOARD;
    if (s.calibrationPattern == Settings::NOT_EXISTING)
        {
            cerr << " Inexistent camera calibration mode: " <<s. patternToUse << endl;
            s.goodInput = false;
        }
    s.atImageList = 0;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;

    object_point=calcBoardCornerPositions(s.boardSize,s.squareSize);

    FileStorage fs("camera_data.xml",FileStorage::READ);
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coeffs"]>>distCoeffs;
    cout<<"Camera instrinsic matrix"<<endl;
    cout<<cameraMatrix<<endl;
    cout<<"camera distortion coefficients"<<endl;
    cout<<distCoeffs<<endl;
    fs.release();

    if(cameraMatrix.empty()) return false;
    if(distCoeffs.empty()) return false;
    return true;


}

Mat  calcBoardCornerPositions(Size boardSize, float squareSize)
{
    int width=boardSize.width;
    int height =boardSize.height;
    Mat corners(width*height,3,CV_32F);
        for( int i = 0; i < boardSize.height; ++i )
           {
            for( int j = 0; j < boardSize.width; ++j )
            {
                corners.at<float>(i*boardSize.width+j,0)=float( j*squareSize );
                corners.at<float>(i*boardSize.width+j,1)=float( i*squareSize );
                corners.at<float>(i*boardSize.width+j,2)=0.0;
            }
          }
        return corners;
}

bool calculate_extrinsic_m(Mat image,Mat *transform,Mat object_point,Mat cameraMatrix,Mat distCoeffs,Settings s )
{

    Size imageSize;
    imageSize=image.size();
    if( s.flipVertical )    flip( image,image, 0 );
    vector<Point2f> pointBuf;
    bool found;
    Mat viewtemp ;
    cvtColor(image,viewtemp,COLOR_BGR2GRAY);
    cout<<"calibration boardsize : "<<s.boardSize<<endl;
    found = findChessboardCorners(image, s.boardSize, pointBuf,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
     if ( found)                // If done with success,
      {
         cout<<"......................found corners......................"<<endl;
         // improve the found corners' coordinate accuracy for chessboard
         if( s.calibrationPattern == Settings::CHESSBOARD)
          {
            Mat viewGray;
            cvtColor(image, viewGray, COLOR_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11),
                         Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
          }
          drawChessboardCorners( image, s.boardSize, Mat(pointBuf), found );

          Mat drawcorner;
          drawcorner=image.clone();
       imwrite("image_coordinate.jpg",drawcorner);
//          imshow("image",drawcorner);
//          waitKey(0);
          Mat  imagepoint(pointBuf.size(),2,CV_32F);
          for(int i=0;i<pointBuf.size();i++)
            {
              imagepoint.at<float>(i,0)=pointBuf[i].x;
              imagepoint.at<float>(i,1)=pointBuf[i].y;
            }

                       CvMat* cv_rotation_vector=cvCreateMat(3,1,CV_32F);
                       CvMat* cv_translation_vector=cvCreateMat(3,1,CV_32F);
                       CvMat cv_object_point=object_point;
                       CvMat cv_imagepoint=imagepoint;
                       CvMat cv_cameraMatrix=cameraMatrix;
                       CvMat cv_distCoeffs=distCoeffs;
                       cvFindExtrinsicCameraParams2(&cv_object_point,&cv_imagepoint,&cv_cameraMatrix,&cv_distCoeffs,cv_rotation_vector,cv_translation_vector);
                       float *rotation1=(float*)cvPtr2D(cv_rotation_vector, 0, 0);
                       float r1,r2,r3;
                       r1=*rotation1;
                       rotation1++;
                       r2=*rotation1;
                       rotation1++;
                       r3=*rotation1;
                       float *translation1=(float*)cvPtr2D(cv_translation_vector, 0, 0);
                       float t1,t2,t3;
                       t1=*translation1;
                       translation1++;
                       t2=*translation1;
                       translation1++;
                       t3=*translation1;
                       Mat rodri(3,1,CV_64F);
                       rodri.at<double>(0,0)=r1;
                       rodri.at<double>(1,0)=r2;
                       rodri.at<double>(2,0)=r3;
                       Mat dcm(3,3,CV_64F);
                       Rodrigues(rodri,dcm);
                       transform->at<double>(0,3)=t1*0.001;
                       transform->at<double>(1,3)=t2*0.001;
                       transform->at<double>(2,3)=t3*0.001;
                       transform->at<double>(3,3)=1;

                       transform->at<double>(3,0)=0;
                       transform->at<double>(3,1)=0;
                       transform->at<double>(3,2)=0;
                       for (int i=0;i<3;i++){
                           for(int j=0;j<3;j++){
                               transform->at<double>(i,j)=dcm.at<double>(i,j);
                           }
                       }
                    return true;

              }
            else
            {
             cout<<"Not found chessboard "<<endl;
             return false;
           }

}
Mat doubleMat2floatMat(Mat t)
{
   int rows  = t.rows;
   int cols = t.cols;
   Mat result(rows,cols,CV_32F);
   for (int i = 0; i<rows;i++)
   {

       for (int j = 0;j<cols;j++)
        result.at<float>(i,j) = t.at<double>(i,j);
   }
   return result;

}

Mat dcm2quat(Mat dcm )
{
    Mat qnb(1,4,CV_64F);
        double temp11=dcm.at<double>(0,0);
        double temp12=dcm.at<double>(0,1);
        double temp13=dcm.at<double>(0,2);
        double temp21=dcm.at<double>(1,0);
        double temp22=dcm.at<double>(1,1);
        double temp23=dcm.at<double>(1,2);
        double temp31=dcm.at<double>(2,0);
        double temp32=dcm.at<double>(2,1);
        double temp33=dcm.at<double>(2,2);
      qnb.at<double>(0,0)=sqrt(abs(1.0 + temp11 + temp22 + temp33))/2.0;
      qnb.at<double>(0,1)=-1.0*sign(temp32-temp23) * sqrt(abs(1.0 + temp11 - temp22 - temp33))/2.0;
      qnb.at<double>(0,2)=-1.0*sign(temp13-temp31) * sqrt(abs(1.0 - temp11 + temp22 - temp33))/2.0;
      qnb.at<double>(0,3)=-1.0*sign(temp21-temp12) * sqrt(abs(1.0 - temp11 - temp22 + temp33))/2.0;
      return qnb;
}

int sign (double i)
{  if(i>0)
        return 1;
    else if(i<0)
        return -1;
    else
        return 0;
 }




