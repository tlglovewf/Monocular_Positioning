
echo "build g2o library"

mkdir ../ThirdParty/g2o/build
cd ../ThirdParty/g2o/build
cmake .. -DCMAKE_BUILD_TYPE=Debug

make -j
echo "build main project"

pwd

cd -
cmake .         
make -j4
