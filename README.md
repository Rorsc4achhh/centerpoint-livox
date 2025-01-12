Key module versiosn listed below:
cuda 11.8;
nvidia-smi driver version 535;
torch==2.4.1;
TensorRT 8.5.1;
cudnn==9.1.0;

Worked with nvidia 4070ti.
More information for env:
(https://github.com/Tream733/centerpoint-livox)




Normal steps to start:

cd ~/ros2_ws

source ~/ros2_ws/install/setup.bash

colcon build --packages-select centerpoint --symlink-install

colcon build --packages-select pcl_reg --symlink-install

ros2 run centerpoint centerpoint_node #detection node

ros2 run pcl_reg reg_pcl #publish node

rosrun rviz rviz
