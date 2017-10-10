/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"

#include <boost/make_shared.hpp>

//using caffe::Blob;
//using caffe::Caffe;
//using caffe::Datum;
//using caffe::Net;
//using caffe::Layer;
//using std::string;
//namespace db = caffe::db;
using namespace std;

//char *labelname[] = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog",
//                      "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );

    /******************************************* caffe ******************************************************************/
//    proto_file="/home/mj/yolo-deploy.prototxt";
//    model_file="/home/mj/yolo.caffemodel";

//    labelname[0]="aeroplane";
//    labelname[1]="bicycle";
//    labelname[2]="bird";
//    labelname[3]="boat";
//    labelname[4]="bottle";
//    labelname[5]="bus";
//    labelname[6]="car";
//    labelname[7]="cat";
//    labelname[8]="chair";
//    labelname[9]="cow";
//    labelname[10]="diningtable";
//    labelname[11]="dog";
//    labelname[12]="horse";
//    labelname[13]="motorbike";
//    labelname[14]="person";
//    labelname[15]="pottedplant";
//    labelname[16]="sheep";
//    labelname[17]="sofa";
//    labelname[18]="train";
//    labelname[19]="tvmonitor";


//    labelname={"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog",
//               "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
    cout<<"....................init caffe ............................."<<endl;
  //  Caffe::set_mode(Caffe::GPU);


  //  boost::shared_ptr<Net<float> > net(new Net<float>(proto_file, caffe::TEST));
    cout<<"........................setting up caffe net ................................"<<endl;

 //   net->CopyTrainedLayersFromBinaryProto(model_file);
//    caffeready = true;

}

void PointCloudMapping::shutdown()
{

    cout<<"..............saving global map............."<<endl;
    pcl::io::savePCDFile( "pointcloud.pcd", *globalMap );
    globalMap->points.clear();
    cout<<"...............Point cloud saved.............."<<endl;
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>6)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            p.r = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.b = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
}

/*************************************about caffe************************************************/
/*
void PointCloudMapping::loadweights(boost::shared_ptr<Net<float> > &net, char *argv)
{

    char txt_name[200];
      strcat(txt_name,argv);
      char path[200];
      const std::vector<boost::shared_ptr<Layer<float> > > layers = net->layers();
      int convolution_n = 0;
      int connect_n = 0;
      FILE* fp;
      char* name = (char*)malloc(sizeof(char)*100);
      boost::shared_ptr<Layer<float> > layer;
      std::vector<boost::shared_ptr<Blob<float> > > blobs;
      for(int i = 0; i < layers.size(); ++i){
        layer = layers[i];
        blobs = layer->blobs();
        if(layer->type() == std::string("Convolution")){
            ++convolution_n;
            std::cout << "convolution" << convolution_n <<std::endl;
            sprintf(path,"%s/convolution%d.txt",argv,convolution_n);
            //std::cout << path << std::endl;
            //sprintf(name,"/home/yang/yolo2caffe/yolo/yolo_convolution%d.txt",convolution_n);
            fp = fopen(path,"r");
            fread(blobs[1]->mutable_cpu_data(), sizeof(float), blobs[1]->count(), fp);
            fread(blobs[0]->mutable_cpu_data(), sizeof(float), blobs[0]->count(), fp);
        }
        else{
          if(layer->type() == std::string("InnerProduct")){
            ++connect_n;
            std::cout << "Connect" << connect_n <<std::endl;
            sprintf(path,"%s/connect%d.txt",argv,connect_n);
            //std::cout << path << std::endl;
            fp = fopen(path,"r");
            fread(blobs[1]->mutable_cpu_data(), sizeof(float), blobs[1]->count(), fp);
            fread(blobs[0]->mutable_cpu_data(), sizeof(float), blobs[0]->count(), fp);
          }
        }
      }
      if(fp != NULL)
        fclose(fp);
      delete []name;
}

void PointCloudMapping::loaddata(boost::shared_ptr<Net<float> > &net, string image_path)
{

    Blob<float>* input_layer = net->input_blobs()[0];
     int width, height;
     width = input_layer->width();
     height = input_layer->height();
     cout<<"image width :"<<width<<endl;
     cout<<"image height :"<<height<<endl;

     int size = width*height;
     cv::Mat image = cv::imread(image_path,-1);
     cv::Mat image_resized;
     cv::resize(image, image_resized, cv::Size(height, width));
   //  cv::imshow("yolo input",image_resized);
   //  cv::waitKey(0);
     float* input_data = input_layer->mutable_cpu_data();
     int temp,idx;
     for(int i = 0; i < height; ++i){
       uchar* pdata = image_resized.ptr<uchar>(i);
       for(int j = 0; j < width; ++j){
         temp = 3*j;
         idx = i*width+j;
         input_data[idx] = (pdata[temp+2]/127.5)-1;
         input_data[idx+size] = (pdata[temp+1]/127.5)-1;
         input_data[idx+2*size] = (pdata[temp+0]/127.5)-1;
       }
     }
     //cv::imshow("image",image_resized);
}

void PointCloudMapping::loaddata_mat(boost::shared_ptr<Net<float> > &net, cv::Mat image)
{
    Blob<float>* input_layer = net->input_blobs()[0];
     int width, height;
     width = input_layer->width();
     height = input_layer->height();

     int size = width*height;
     cv::Mat image_resized;
     cv::resize(image, image_resized, cv::Size(height, width));
   //  cv::imshow("yolo input",image_resized);
   //  cv::waitKey(0);
     float* input_data = input_layer->mutable_cpu_data();
     int temp,idx;
     for(int i = 0; i < height; ++i){
       uchar* pdata = image_resized.ptr<uchar>(i);
       for(int j = 0; j < width; ++j){
         temp = 3*j;
         idx = i*width+j;
         input_data[idx] = (pdata[temp+2]/127.5)-1;
         input_data[idx+size] = (pdata[temp+1]/127.5)-1;
         input_data[idx+2*size] = (pdata[temp+0]/127.5)-1;
       }
     }
     //cv::imshow("image",image_resized);
}

void PointCloudMapping::getbox(std::vector<float> result, float *pro_obj, int *idx_class, std::vector<std::vector<int> > &bboxs, float thresh, cv::Mat image)
{
    float pro_class[49];
      int idx;
      float max_idx;
      float max;
      for(int i = 0; i < 7; ++i){
        for(int j = 0; j < 7;++j){
          max = 0;
          max_idx = 0;
          idx = 20*(i*7+j);
          for(int k = 0; k < 20; ++k){
            if (result[idx+k] > max){
              max = result[idx+k];
              max_idx = k+1;
            }
          }
          idx_class[i*7+j] = max_idx;
          pro_class[i*7+j] = max;
          pro_obj[(i*7+j)*2] = max*result[7*7*20+(i*7+j)*2];
          pro_obj[(i*7+j)*2+1] = max*result[7*7*20+(i*7+j)*2+1];
        }
      }
      std::vector<int> bbox;
      int x_min,x_max,y_min,y_max;
      float x,y,w,h;
      for(int i = 0; i < 7;++i){
        for(int j = 0; j < 7;++j){
          for(int k = 0; k < 2; ++k){
              if(pro_obj[(i*7+j)*2 + k] > thresh){
                  //std::cout << "(" << i << "," << j << "," << k << ")" << " prob="<<pro_obj[(i*7+j)*2 + k] << " class="<<idx_class[i*7+j]<<std::endl;
                  idx = 49*20 + 49*2 + ((i*7+j)*2+k)*4;
                  x = image.cols*(result[idx++]+j)/7;
                  y = image.rows*(result[idx++]+i)/7;
                  w = image.cols*result[idx]*result[idx++];
                  h = image.rows*result[idx]*result[idx];
                  //std::cout << x <<" "<< y << " " << w <<" "<< h <<std::endl;
                  x_min = x - w/2;
                  y_min = y - h/2;
                  x_max = x + w/2;
                  y_max = y + h/2;
                  bbox.clear();
                  bbox.push_back(idx_class[i*7+j]);
                  bbox.push_back(x_min);
                  bbox.push_back(y_min);
                  bbox.push_back(x_max);
                  bbox.push_back(y_max);
                  bbox.push_back(int(pro_obj[(i*7+j)*2 + k]*100));
                  bboxs.push_back(bbox);
              }
          }
        }
      }

}


*/
