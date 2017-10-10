#include "setting.h"

Settings::Settings() {}

void Settings::write(FileStorage &fs) const
{

    fs << "{" << "BoardSize_Width"  << boardSize.width
              << "BoardSize_Height" << boardSize.height
              << "Square_Size"         << squareSize
              << "Calibrate_Pattern" << patternToUse
              << "Calibrate_NrOfFrameToUse" << nrFrames
              << "Calibrate_FixAspectRatio" << aspectRatio
              << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
              << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

              << "Write_DetectedFeaturePoints" << bwritePoints
              << "Write_extrinsicParameters"   << bwriteExtrinsics
              << "Write_outputFileName"  << outputFileName

              << "Show_UndistortedImage" << showUndistorsed

              << "Input_FlipAroundHorizontalAxis" << flipVertical
              << "Input_Delay" << delay
              << "Input" << input
       << "}";
}

 bool Settings::readStringList(const string &filename, vector<string> &l)
{
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
//    if( n.type() != FileNode::SEQ )
//        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;


}
