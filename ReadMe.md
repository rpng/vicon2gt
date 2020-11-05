# vicon2gt


Uses Vicon as groundtruth and calculates the IMU state (full 15 dof) at each camera time.
Used to get groundtruth trajectories simular to those provided by the EurocMav dataset.
To run please take a look at the example launch files.
You need have a bag dataset that has the IMU, camera, and Vicon measurements in it.
The Vicon can publish either `nav_msgs/Odometry`, `geometry_msgs/PoseStamped`, or `geometry_msgs/TransformStamped`.
If you are using the odometry topic it will use the provided covariance, otherwise it will use the one specified in the launch file.
If the system has trouble converging, consider fixing the magnitude of gravity or checking that your noise levels are correct.
Documentation and derivations are forthcoming.



## Building GTSAM

Ensure that you have Intel TBB installed as this will allow for GTSAM multithreaded performance ([link](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo)).
You don't need this, but this will allow for faster optimization.
```cmd
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sudo sh -c 'echo deb https://apt.repos.intel.com/tbb all main > /etc/apt/sources.list.d/intel-tbb.list'
sudo apt-get update
sudo apt-get install intel-tbb-2020.3-912
```

Then we can [build](https://gtsam.org/get_started/) [GTSAM](https://gtsam.org/build/) as normal and install it globally on our system.
Note that we use the system Eigen since we want to link with packages which also use that Eigen version.
```cmd
sudo apt install libboost-all-dev libeigen3-dev libmetis-dev
git clone https://github.com/borglab/gtsam
mkdir gtsam/build/
cd gtsam/build/
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j6
sudo make -j6 install
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
```








