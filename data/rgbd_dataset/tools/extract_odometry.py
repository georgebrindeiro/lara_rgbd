#!/usr/bin/env python

import rosbag

filename = 'rgbd_dataset_freiburg2_pioneer_360'

bag = rosbag.Bag('../datasets/'+filename+'.bag')
f = open('../datasets/'+filename+'-odometry.txt', 'w')

f.write("# odometry\n")
f.write("# file: " + repr(filename) + "\n")
f.write("# timestamp tx ty tz qx qy qz qw\n")

for topic, msg, t in bag.read_messages(topics=['/tf']):
    for tr in msg.transforms:
        if tr.header.frame_id == '/odom':
            timestamp = tr.header.stamp.secs + 1e-9*tr.header.stamp.nsecs
            tx = tr.transform.translation.x
            ty = tr.transform.translation.y
            tz = tr.transform.translation.z
            qx = tr.transform.rotation.x
            qy = tr.transform.rotation.y
            qz = tr.transform.rotation.z
            qw = tr.transform.rotation.w

            print >> f, '%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (timestamp, tx, ty, tz, qx, qy, qz, qw)

bag.close()
