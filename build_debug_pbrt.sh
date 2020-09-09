rm -rf debug_build/*
y
cd debug_build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
