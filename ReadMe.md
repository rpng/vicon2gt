# vicon2gt

This utility was created to generate groundtruth trajectories using a motion capture system (e.g. Vicon or OptiTrack) for use in evaluating visual-inertial estimation systems.
Specifically we want to calculate the inertial IMU state (full 15 dof) at camera frequency rate and generate a groundtruth trajectory similar to those provided by the EurocMav datasets.
Please take a look at the **[REPORT](docs/report.pdf)** for specific details and a more formal description.

You will need to have a bag dataset that has the IMU, camera, and motion capture measurements of either `geometry_msgs::TransformStamped`, `geometry_msgs::PoseStamped`, or `nav_msgs::Odometry`.
If you are using the odometry topic it will use the provided covariance, otherwise it will use the one specified in the launch file.
To run please take a look at the example launch files and try them out before testing on your own dataset.
ros

## Example Outputs


![trajectory optimized](docs/traj.png)

```
state_0: 
m_time:[1413394882.44]'
q:[0.0257068648823, -0.795603373718, 0.0363500247023, 0.604179695563]'
bg:[-0.00189916154709, 0.0243152833864, 0.0807307477958]'
v:[-0.000133337500769, 0.00575180823653, -0.000724871724463]'
ba:[0.0190844911612, 0.091082677377, 0.00706744868603]'
p:[-1.05876790375, 0.428205735887, 1.33507858186]'

state_N: 
m_time:[1413394997.14]'
q:[-0.492577574755, -0.637052268672, -0.359427482017, 0.4715332703]'
bg:[-0.00161559034645, 0.0255108277786, 0.079954020838]'
v:[-0.00726564037527, 0.012639815567, -0.00249341953281]'
ba:[0.0311904335341, 0.0279113668375, 0.0250458337156]'
p:[-3.02776826575, -0.526174099205, 0.96038346419]'

R_BtoI: 
 0.301686577941 0.0177220217954  0.953242434344
0.0246935408972 -0.999637058319 0.0107694323235
 0.953087319047 0.0202899378534 -0.302014702778

p_BinI: 
 0.0765661266578 -0.0237148659109 -0.117918182433

gravity: 
0.0743268225068 0.0849888222512 9.76992397331

t_off_vicon_to_imu: 
0.196106089294
```



## Frequently Asked Questions

1) *Help, doesn't converge to a good trajectory!* -- This can be caused by many things and is hard to debug. The first thing we recommend doing is to look at the optimized trajectory in RVIZ and see if the alignment there is ok. If there is issues with a noisy poses you will see it in RVIZ. In this case it is likely you will not be able to optimize the trajectory. Another source might be poor noise values, try to play with both the IMU noises and vicon pose noises. If the dataset is degenerate, you might want to try fixing the marker to IMU transform (see launch files) as if there is not enough motion this can bias the trajectory results.

2) *Do I need an initial guess of the marker body to IMU?* -- We found that on good datasets you can just set these to be an identity transformation. If you know these values you can also try fixing and not estimating them online. Additionally if there is enough rotation in the dataset the time offset usually robustly converges.

3) *The calibration seems to not converge to the same thing on different collected datasets, why?* -- We don't expect the calibration to converge to the same thing on each dataset as this is highly depends on the motion of the trajectory. If you worry about it you can try fixing it and not optimizing it, but in general even if the calibration is different the trajectory itself should still be of high quality.

4) *Do I need a camera topic?* -- No, but you need some topic to specify what timestamps you want the optimized states to be at. We recommend you specify anything lower than the IMU frequency, thus you could specify the vicon topic, or another topic.

5) *Explain the timestamps the groundtruth file has in it* -- We have two time systems in the project: vicon and inertial. The states we estimate in the optimization problem are in the IMU clock frame and the CSV file we save has timestamps in the IMU clock frame. We use an arbitrary topic timestamps to define what timestamps we will export, but all these times are still in the IMU clock frame. E.g. if we use a camera topic, the CSV will have the poses at the IMU clock time of the timestamps in this topic (i.e. if you wish to get the pose at the camera timestamp you will have an additional imu-to-camera time offset you need to worry about and is not taken into account here).

6) *What frame of reference is the groundtruth file in?* -- The trajectory is in the motion capture frame of reference which is not necessarily gravity aligned. Additionally we saw in simulation that in many cases with high noise or degeneracy there was a trajectory position bias (e.g. off by 1-2cm due to vicon marker to IMU calibration offset), thus it is recommended to perform a full SE(3) alignment to your visual-inertial trajectory.





## Building GTSAM and Dependencies

GTSAM is the only package you need to build the others (boost, eigen3) typically come on a linux-based OS.
You can optionally install Intel TBB installed as this will allow for GTSAM multithreaded performance ([link](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo)).
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
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j6
sudo make -j6 install
echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
```






