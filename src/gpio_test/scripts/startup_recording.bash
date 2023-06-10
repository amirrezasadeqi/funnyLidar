#!/usr/bin/env bash


# add this to your home directory and also crontab file.

source /home/lidarpi/Documents/ws_livox/devel/setup.bash
roscore &
rosrun gpio_test playGround_node.py --launch_path "$(rospack find gpio_test)/launch/playGround.launch"
#rosrun gpio_test playGround_node.py --launch_path "$(rospack find gpio_test)/launch/livox_mapping_test.launch"
#rosrun gpio_test playGround_node.py --launch_path "$(rospack find gpio_test)/launch/fast_lio_test.launch"
