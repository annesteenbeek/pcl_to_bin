# Simple ros node to convert pointcloud2 into KITTI format
Ros uses the PointCloud2 format, whereas a lot of other systems use the KITTI bin format

In order to use these tools with ROS, it might be useful to convert them.


## Dependencies
The only dependency is ros-numpy
```
$ sudo apt install ros-noetic-ros-numpy
```
Then install as a regular catkin package



## Usage
First start Roscore
```
$ roscore
```
In another terminal start playing a bag (or anything that publishes a PointCloud2 topic)
```
$ rosbag play $BAGNAME
```

Then run the script
```
$ rosrun pcl_to_bin pcl_to_bin.py --help                 

usage: pcl_to_bin.py [-h] [--out OUT] interval topic

Convert PointCloud2 messages into kitti bin format

positional arguments:
  interval    interval at which to subscribe to messages
  topic       Topic to listen to for pcl messages

optional arguments:
  -h, --help  show this help message and exit
  --out OUT   Output directory
```
