# mjd_ego_planner


onboardsdk3.6
onboardsdk ros 3.6。0
cd Onboard-SDK 
mkdir build
cd build
cmake ..
make djiosdk-core
sudo make install djiosdk-core


ceres 
cd crers
    mkdir build
    cd build
    cmake ..
    make 


sudo apt-get install ros-kinetic-ddynamic-reconfigure


realsense2 missing
参考
https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md


rosrun global_fusion global_fusion_node 
rosrun vins vins_node /home/dji/tfes/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
rosrun global_fusion global_fusion_node 


evo 
配置的时候可能出现相关报错，可以通过注释相关代码解决
evo_res gps_rtk.zip fusion_rtk.zip  -p
//tum_rtk.txt作为reference
evo_ape tum tum_rtk.txt  tum_gps.txt   -r trans_part -vs --plot --plot_mode xy --save_results ./gps_rtk.zip
evo_ape tum tum_rtk.txt  tum_fusion.txt   -r trans_part -vs --plot --plot_mode xy --save_results ./fusion_rtk.zip
evo_traj tum tum_rtk.txt tum_gps.txt --ref=tum_rtk.txt -p --plot_mode=xy -s



