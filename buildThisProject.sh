echo "Prerequisites:"
echo "Pangolin, OpenCV, Eigen3, DBow2, g2o, ROS-Kinetic"

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building Thirdparty/Pangolin ..."
cd ../../Pangolin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building ORB_SLAM2 ..."
cd ../../../
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
