# -*- coding: utf-8 -*-
# @Author: Heethesh Vhavle
# @Date:   Nov 20, 2019
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Nov 20, 2019

import os

import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

from ti_mmwave_rospkg.msg import RadarScan, RadarScanArray


class BagConverter:
    def __init__(self, old_filename, new_filename, write=False, fix_tf=False):
        self.write = write
        self.fix_tf = fix_tf
        self.bag = rosbag.Bag(old_filename)
        
        if self.write:
            self.new_bag = rosbag.Bag(new_filename, 'w')
            self.copy_topics()

        self.radar_scan_topic = '/ti_mmwave/radar_scan'
        self.radar_scan_pcl_topic = '/ti_mmwave/radar_scan_pcl'
        self.radar_scan_array_topic = '/ti_mmwave/radar_scan_array'

        self.radar_scan_count = self.bag.get_message_count(self.radar_scan_topic)
        self.radar_scan_pcl_count = self.bag.get_message_count(self.radar_scan_pcl_topic)
        self.radar_scan_pcl_count = self.bag.get_message_count(self.radar_scan_pcl_topic)

        self.min_point_count = 20
        assert self.radar_scan_count > 0, 'RadarScan messages not found'
        print('Found %d RadarScan messages' % self.radar_scan_count)
        print('Found %d RadarScan PCL messages' % self.radar_scan_pcl_count)

    def copy_topics(self):
        if self.write:
            if self.fix_tf:
                self.fix_rc_car_frame()
            else:
                for topic, msg, timestamp in self.bag.read_messages():
                    self.new_bag.write(topic, msg, timestamp)

    def convert(self):
        msg_count = 0
        last_point_id = -1
        point_id_count = 0
        point_counts = []
        first = True
        base_time = None
        array_msg = RadarScanArray()
        scan_timestamps = []

        for topic, msg, timestamp in self.bag.read_messages(topics=self.radar_scan_topic):
            if first:
                base_time = timestamp.to_nsec()
                first = False

            if last_point_id + 1 == msg.point_id:
                point_id_count += 1
                scan_timestamps.append(timestamp.to_nsec())
                array_msg.scans.append(msg)
            else:
                if point_id_count >= self.min_point_count:
                    avg_nsecs = np.mean(scan_timestamps)
                    new_timestamp = rospy.Time(int(avg_nsecs // 1e9), int(avg_nsecs % 1e9))
                    point_counts.append([avg_nsecs - base_time, point_id_count])
                    array_msg.header.frame_id = msg.header.frame_id
                    array_msg.header.stamp = new_timestamp
                    if self.write:
                        self.new_bag.write(self.radar_scan_array_topic, array_msg, new_timestamp)

                # Reset
                last_point_id = 0
                point_id_count = 1
                array_msg = RadarScanArray()
                array_msg.scans.append(msg)
                scan_timestamps = [timestamp.to_nsec()]

            last_point_id = msg.point_id

            # msg_count += 1
            # if msg_count > 50: return

        msg_count = 0
        pcl_counts = []
        first = True
        base_time = None

        for topic, msg, timestamp in self.bag.read_messages(topics=self.radar_scan_pcl_topic):
            if first:
                base_time = timestamp.to_nsec()
                first = False

            pcl_counts.append([timestamp.to_nsec() - base_time, msg.width])

            # msg_count += 1
            # if msg_count > 1: return

        print(len(point_counts), self.radar_scan_pcl_count)
        point_counts, pcl_counts = np.array(point_counts), np.array(pcl_counts) 
        # plt.plot(point_counts[:, 0], point_counts[:, 1])
        # plt.plot(pcl_counts[:, 0], pcl_counts[:, 1])
        # plt.grid()
        # plt.show()

    def fix_rc_car_frame(self):
        if not self.write: print('Not updating TF (read-only mode)')
        print('TF count before:', self.bag.get_message_count('/tf'))

        for topic, msg, timestamp in self.bag.read_messages():
            if topic == '/tf' and msg.transforms[0].child_frame_id == 'rc_car':
                msg.transforms[0].transform.rotation.x = 0.0
                msg.transforms[0].transform.rotation.y = 0.0
                msg.transforms[0].transform.rotation.z = 0.0
                msg.transforms[0].transform.rotation.w = 1.0
                self.new_bag.write('/tf', msg, timestamp)
            else:
                self.new_bag.write(topic, msg, timestamp)

        print('TF count after:', self.new_bag.get_message_count('/tf'))

    def close(self):
        self.bag.close()
        if self.write: self.new_bag.close()


if __name__ == '__main__':
    bag_folder = '/mnt/data/Datasets/OptiTrack/dynamic_walking'
    target_folder = os.path.join(bag_folder, 'converted')
    if not os.path.isdir(target_folder): os.makedirs(target_folder)
    bag_files = os.listdir(bag_folder)
    
    for bag_file in tqdm(bag_files):
        if not bag_file.endswith('.bag'): continue
        old_path = os.path.join(bag_folder, bag_file)
        new_path = os.path.join(target_folder, bag_file)

        bag_converter = BagConverter(old_path, new_path, write=True, fix_tf=True)
        bag_converter.convert()
        bag_converter.close()
