mkdir /github/workspace/build

cmake -D CMAKE_BUILD_TYPE=Release -S /github/workspace -B /github/workspace/build
make -j4 -C /github/workspace/build