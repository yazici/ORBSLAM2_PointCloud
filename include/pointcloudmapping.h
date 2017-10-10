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

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>

#include <string>
#include <vector>

#include "boost/algorithm/string.hpp"
#include "google/protobuf/text_format.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


//#include "caffe/blob.hpp"
//#include "caffe/layer.hpp"
//#include "caffe/common.hpp"
//#include "caffe/net.hpp"
//#include "caffe/proto/caffe.pb.h"
//#include "caffe/util/db.hpp"
//#include "caffe/util/format.hpp"
//#include "caffe/util/io.hpp"
//#include <stdio.h>
//#include <malloc.h>
//#include <fstream>
//#include <boost/progress.hpp>
//#include <iostream>

//using caffe::Blob;
//using caffe::Caffe;
//using caffe::Datum;
//using caffe::Net;
//using caffe::Layer;
//using std::string;
//namespace db = caffe::db;

using namespace ORB_SLAM2;

//template<typename Dtype>
//Dtype lap(Dtype x1_min,Dtype x1_max,Dtype x2_min,Dtype x2_max){
//    if(x1_min < x2_min){
//        if(x1_max < x2_min){
//            return 0;
//        }else{
//            if(x1_max > x2_min){
//                if(x1_max < x2_max){
//                    return x1_max - x2_min;
//                }else{
//                    return x2_max - x2_min;
//                }
//            }else{
//                return 0;
//            }
//        }
//    }else{
//        if(x1_min < x2_max){
//            if(x1_max < x2_max)
//                return x1_max-x1_min;
//            else
//                return x2_max-x1_min;
//        }else{
//            return 0;
//        }
//    }
//}


class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping( double resolution_ );
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();
    bool    shutDownFlag    =false;
    bool    caffeready      =false;




protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;


    mutex   shutDownMutex;

    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;

    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;

    //about caffe
//    char *labelname[];
//    string proto_file;
//    string model_file;
//    boost::shared_ptr<Net<float> > net;
//    void loadweights(boost::shared_ptr<Net<float> >& net,char* argv);
//    void loaddata(boost::shared_ptr<Net<float> >& net, std::string image_path);
//    void loaddata_mat(boost::shared_ptr<Net<float> >& net, cv::Mat image);
//    void getbox(std::vector<float> result,float* pro_obj,int* idx_class,std::vector<std::vector<int> >& bboxs,float thresh,cv::Mat image);


};

#endif // POINTCLOUDMAPPING_H
