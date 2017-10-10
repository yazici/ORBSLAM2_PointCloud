
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


//std c++

#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <fstream>
#include<System.h>
using namespace std;
using namespace cv;
string datasetpath;
string getnextrgb(ifstream &fin,string datasetpath);
string getnextdepth(ifstream &fin,string datasetpath);
string findfilename_depth(char* st,string datasetpath);
string findfilename_rgb(char* st,string datasetpath);
void txt_init(ifstream &fin,ifstream & find);

class ParameterReader
{
public:
    ParameterReader( string filename="./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

char line[1024]={0};
char lined[1024]={0};
int main(int argc, char** argv){

    if(argc!=3)
    {
     return -1;
    }

 cout<<"start... "<<endl<<endl;
/*reading parameters */
    ParameterReader pd;
    string  rgbtxt=pd.getData("rgbtxt");
    string  depthtxt=pd.getData("depthtxt");
    datasetpath=pd.getData("dataset_path");
/*initialization*/
       std::ifstream fin(rgbtxt.c_str(), std::ios::in);
       std::ifstream find(depthtxt.c_str(), std::ios::in);
       txt_init(fin,find);


/*reading image*/

/*tips : rgbfile format CV_8UC3*/

       ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);


      int count = 3000;

//      while( !SLAM.mpPointCloudMapping->caffeready)
//      {
//             usleep(5000);

//      }
//      cout<<"............caffe is ready..................."<<endl;

      while(count--)
      {

      string rgbfilename=getnextrgb(fin,datasetpath);
      string  depthfilename=getnextdepth(find,datasetpath);

      Mat rgb=imread(rgbfilename,CV_LOAD_IMAGE_COLOR);
      Mat depth=imread(depthfilename,-1);

      cout<<rgbfilename<<endl;
      cout<<depthfilename<<endl;

      cout<<"test2"<<endl;
      SLAM.TrackRGBD(rgb,depth,0.3000-count);
      cout<<"test3"<<endl;

      }
      SLAM.Shutdown();
      SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
      SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");



   fin.clear();
   fin.close();
   find.clear();
   find.close();

}
void txt_init(ifstream &fin,ifstream & find)
{
    char line[1024]={0};
    for(int i=0;i<3;i++){
       fin.getline(line, sizeof(line));
       find.getline(line, sizeof(line));
    }
}
string getnextrgb(ifstream &fin,string datasetpath)
{
 char line[1024]={0};
 fin.getline(line,sizeof(line));
 string rgbfilename=findfilename_rgb(line,datasetpath);
 return rgbfilename;
}

string getnextdepth(ifstream &fin,string datasetpath)
{


 char line[1024]={0};
 fin.getline(line,sizeof(line));
 string depthfilename=findfilename_depth(line,datasetpath);
 return depthfilename;
}


string findfilename_rgb(char* st,string datasetpath)
{
     string str=st;
    if ("" == str)
    {
        return str;
    }
    string pattern="/";
    size_t pos = str.find(pattern);
    size_t size = str.size();
    str = str.substr(pos+1,size);
    string rgbpath=datasetpath+string("rgb/");
//    string  rgbpath="/home/mj/dataset_person/rgb/";
    rgbpath=rgbpath+str;

    return rgbpath;
}
string findfilename_depth(char* st,string datasetpath)
{
    string str=st;
    if ("" == str)
    {
        return str;
    }
    string pattern="/";
    size_t pos = str.find(pattern);
    size_t size = str.size();
    str = str.substr(pos+1,size);
    string depthpath=datasetpath+string("depth/");
//    string  depthpath="/home/mj/dataset_person/depth/";
    depthpath=depthpath+str;

    return depthpath;

}
