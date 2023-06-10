## A package to Start and Shutdown the launch files in tests

This package has a node, `scripts/playGround_node.py`, which is used to capture falling edge
signals from a push button attached to raspberry pi GPIO pinouts. presseing the button launches
ROS launch files in the launch directory of this package. pressing the button again, shuts down
the launched nodes. This is used to record test data and maps during test fields of funnyLidar.

Things that can be done by pressing the button are:

1. Saving the data published by livox_ros_driver in a bag file under `~/.ros` directory.

2. Saving the 3D map captured by livox_mapping package, under `livox_pcd_data` directory of the
   livox_mapping package.

3. Saving the 3D map captured by hku_mars/FAST_LIO package under `PCD` directory of this package.

Note that you should have installed and sourced the necessary packages of these ROS packages and also
some python and ubuntu packages like, `RPi.GPIO, rpi.gpio-common`.

To change the job which is done by pressing the button, you should uncomment the line corresponding to
your work in the `~/startup_recording.bash` file. This file must be executable and added to the crontab
by running `crontab -e` command, like below:

```bash
# end of cron table file.
@reboot /home/lidarpi/startup_recording.bash
```

Note: It is better to have installed all the related ROS packages and this package under the same workspace
, because in the `startup_recording.bash` file we have sourced only the workspace of gpio_test.

Enjoy that!
