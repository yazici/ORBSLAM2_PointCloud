#ifndef SETTING_H
#define SETTING_H
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif
using namespace cv;
using namespace std;


class Settings
{
public:
//    Settings():goodInput(false) {}
    Settings();
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};
    void write(FileStorage& fs) const;
     bool readStringList(const string &filename, vector<string> &l);
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->
    string patternToUse;


    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;


};

#endif // SETTING_H
