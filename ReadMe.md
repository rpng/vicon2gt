# vicon2gt


Uses Vicon as groundtruth and calculates the IMU state (full 15 dof) at each camera time.
Used to get groundtruth trajectories simular to those provided by the EurocMav dataset.
Documentation and derivations are forthcoming.



## Building GTSAM

Ensure that you have Intel MKL and Intel TBB installed as this will allow for GTSAM multithreaded performance ([link](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo
)):
```cmd
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'
sudo sh -c 'echo deb https://apt.repos.intel.com/tbb all main > /etc/apt/sources.list.d/intel-tbb.list'
sudo apt-get update
sudo apt-get install intel-mkl-2018.1-038 intel-mkl-2019.2-057
sudo apt-get install intel-tbb-2018.0-033
```

Then we can build GTSAM as normal and install it globally on our system.
```cmd
git clone https://bitbucket.org/gtborg/gtsam/
cd gtsam
mkdir build
cd build
cmake ..
sudo make -j6 install
```








