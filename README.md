# mjd_ego_planner


<<<<<<< HEAD
=======
配置github
本地仓库默认为master
远程仓库为main
同步本地仓库为main
https://blog.csdn.net/qq_45531729/article/details/111668830
上传最原始版本src

本地仓库设为master，用于修改代码，版本控制
查看仓看
git barnch git
切换仓库
git checkout name
本地放到暂存区
git add .
本地提交到仓库
git commit -m "修改信息"
同步到远程
git remote add origin git@github.com:DoubleMuch/mjd_ego_planner.git
git push -u origin master（分支名字）



>>>>>>> 85a1afe98e428291215883afc8b7929ab419191e
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


