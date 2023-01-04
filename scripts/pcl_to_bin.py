#!/usr/bin/env python3

import rospy
import numpy as np
import copy
import argparse
from sensor_msgs.msg import PointCloud2
import ros_numpy
from threading import Thread
import time
import os


latest_msg: PointCloud2 = None 
file_count = 0


def store_array(arr: np.ndarray):
  global file_count
  outdir = os.path.join(args.out, 'pointclouds')
  if not os.path.isdir(outdir):
    os.makedirs(outdir)
  filename = os.path.join(outdir, "pointcloud{:05d}.bin".format(file_count))
  file_count+=1
  Thread(target = arr.astype('float32').tofile, args=(filename,), daemon=True).start()
  print(f"Writing file {filename}")


def convert_pointcloud():
  tic = time.perf_counter()
  if latest_msg is None:
    return
  msg = copy.deepcopy(latest_msg)

  pc = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
  # remove nans
  mask = np.isfinite(pc['x']) & np.isfinite(pc['y']) & np.isfinite(pc['z']) & np.isfinite(pc['intensity'])
  pc = pc[mask]

  x = pc['x'].flatten()
  y = pc['y'].flatten()
  z = pc['z'].flatten()
  intensity = pc['intensity']
  arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
  arr[::4] = x
  arr[1::4] = y
  arr[2::4] = z
  arr[3::4] = intensity

  toc= time.perf_counter()
  # print(f"function time: {toc - tic:0.4f} seconds")
  store_array(arr)


def callback_pointcloud(msg):
  global latest_msg
  latest_msg = msg

def _dir_path(string):
    if os.path.isdir(string):
        return string
    else:
        raise NotADirectoryError(string)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Convert PointCloud2 messages into kitti bin format')
  parser.add_argument('interval', type=float, default=1, help='interval at which to subscribe to messages')
  parser.add_argument('topic', type=str, default='/pcl', help='Topic to listen to for pcl messages')
  parser.add_argument('--out', type=_dir_path, default=os.getcwd(), help="Output directory")

  args = parser.parse_args()

  rospy.init_node('pcl_to_bin', anonymous=True)
  rospy.Subscriber(args.topic, PointCloud2, callback=callback_pointcloud, queue_size=1)
  
  while not rospy.is_shutdown():
    convert_pointcloud()
    rospy.sleep(args.interval)
