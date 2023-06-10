#!/usr/bin/env python3

import time
import signal
import RPi.GPIO as GPIO
import rospy
import roslaunch
import sys
import argparse

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def button_pressed_callback(channel):
    global recording
    global serve_the_key
    serve_the_key = True
    if not recording:
        recording = True
    else:
        recording = False

button_gpio = 23

if __name__ == "__main__":

    rospy.init_node("playGround_node")
    parser = argparse.ArgumentParser()
    parser.add_argument("-L", "--launch_path", help="Absolute path to the launch file which is running by push button.")
    args = parser.parse_args()
    launch_path = args.launch_path if (args.launch_path) else "/home/lidarpi/Documents/ws_livox/src/gpio_test/launch/playGround.launch"
    # flag to specify if we want to start or stop recording
    global recording
    recording = False
    # another flag to prevent launching the launch file in the main
    # while loop
    global serve_the_key
    serve_the_key = False

    # setup the push button to start and stop the recording procedure
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(button_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(button_gpio, GPIO.FALLING, callback=button_pressed_callback, bouncetime=200)

    signal.signal(signal.SIGINT, signal_handler)
    while not rospy.is_shutdown():
        if serve_the_key:
            serve_the_key = False
            if recording:

                # setting up the launch file executor "launch"
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
                rospy.loginfo("Start Recording")
                launch.start()
                #rospy.sleep(3)
                rospy.loginfo("Recording Started")

            else:

                rospy.loginfo("Stop Recording")
                launch.shutdown()
                #rospy.sleep(3)
                rospy.loginfo("Recording Stopped")



