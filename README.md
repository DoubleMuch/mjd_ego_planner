# mjd_ego_planner


·  git add -A  提交所有变化


·  git add -u  提交被修改(modified)和被删除(deleted)文件，不包括新文件(new)


·  git add .  提交新文件(new)和被修改(modified)文件，不包括被删除(deleted)文件


本地仓库默认为master

远程仓库为main


同步本地仓库为main


https://blog.csdn.net/qq_45531729/article/details/111668830


上传最原始版本src


本地仓库设为master，用于修改代码，版本控制


查看仓看git barnch git


切换仓库git checkout name


本地放到暂存区git add .


本地提交到仓库git commit -m "修改信息"


同步到远程git remote add origin git@github.com:DoubleMuch/mjd_ego_planner.git


git push -u origin master（分支名字）



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



