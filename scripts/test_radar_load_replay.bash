#!/bin/bash
rosbag play test_radar_bag.bag radar_packet:=radar_packet/can0/recv > /dev/null &
rosbag play test_radar_bag.bag radar_packet:=radar_packet/can1/recv > /dev/null &
rosbag play test_radar_bag.bag radar_packet:=radar_packet/can2/recv > /dev/null &
rosbag play test_radar_bag.bag radar_packet:=radar_packet/can3/recv > /dev/null &
rosbag play test_radar_bag.bag radar_packet:=radar_packet/can4/recv > /dev/null &
