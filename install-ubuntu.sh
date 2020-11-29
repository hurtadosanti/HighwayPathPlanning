#! /bin/bash
sudo apt-get install libuv1-dev libssl-dev libz-dev
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
#Install Eigen
wget -nv https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz \
tar xvfz eigen-3.3.8.tar.gz
cd eigen-3.3.8
mkdir build && cd build && cmake .. && make install