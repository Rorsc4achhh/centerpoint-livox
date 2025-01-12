# livox_data_loader: load livox dataset and publish by sensor msg on ros2  

livox_data_loader is a tool for replaying livox datasets, which can replay the data in the dataset into ros2 msg.

# Data
At first: you need to download the [LivoxOpenDataSet](https://www.livoxtech.com/cn/dataset).
Then： build this package in an ros2 project and change the path of livoxdataset.
Now: you can run the project with the following command：
  ‘’‘
     source ./install/setup.bash
     ros2 launch data_loader data_loader_launch.py
  ’‘’

# Message
| SENSOR  |   MODEL   |  LOCATION  |   TOPIC NAME                |   MESSAGE TYPE                |
| ------- | --------- | ---------- | --------------------------- | ----------------------------- |
|  lidar  | Horizon   |   Front    |livox/point_cloud_front      | sensor_msgs::msg::PointCloud2 |
|  lidar  | Horizon   | Left front |livox/point_cloud_left_front | sensor_msgs::msg::PointCloud2 |
|  lidar  | Horizon   | Left rear  |livox/point_cloud_left_rear  | sensor_msgs::msg::PointCloud2 |
|  lidar  | Horizon   | Right rear |livox/point_cloud_right_rear | sensor_msgs::msg::PointCloud2 |
|  lidar  | Horizon   | Right front|livox/point_cloud_right_front| sensor_msgs::msg::PointCloud2 |
|  lidar  | Tele-15   |   Tele     |    livox/point_cloud_tele   | sensor_msgs::msg::PointCloud2 |
|  camera | Camera    |   Front    |     livox/image_front       |    sensor_msgs::msg::Image    |
|  camera | Camera    | Left front |   livox/image_left_front    |    sensor_msgs::msg::Image    |
|  camera | Camera    | Left rear  |   livox/image_left_rear     |    sensor_msgs::msg::Image    |
|  camera | Camera    | Right rear |  livox/image_right_rear     |    sensor_msgs::msg::Image    |
|  camera | Camera    | Right front|   livox/image_right_front   |    sensor_msgs::msg::Image    |
|  camera | Long focal|    Front   |      livox/image_tele       |    sensor_msgs::msg::Image    |
|   gt    |Lidar label|    Front   |        livox/gt             |    ws_msgs::msg::BboxArray    |

# Time
publish frequency：10HZ
message header stamp： git it from filename
