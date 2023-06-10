#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import signal
import sys


def start_btn_pressed_cb(channel):
    print("Start Button Pressed!")

def stop_btn_pressed_cb(channel):
    print("Stop Button Pressed!")

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

start_btn_gpio = 23
stop_btn_gpio = 24

if __name__ == "__main__":
    rospy.init_node("gpio_test_node")
    rospy.loginfo("Hello from gpio node")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(start_btn_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(stop_btn_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(start_btn_gpio, GPIO.IN)
    #GPIO.setup(stop_btn_gpio, GPIO.IN)

    GPIO.add_event_detect(start_btn_gpio, GPIO.FALLING, callback=start_btn_pressed_cb, bouncetime=400)
    GPIO.add_event_detect(stop_btn_gpio, GPIO.FALLING, callback=stop_btn_pressed_cb, bouncetime=400)

    signal.signal(signal.SIGINT, signal_handler)

    while not rospy.is_shutdown():
        pass

